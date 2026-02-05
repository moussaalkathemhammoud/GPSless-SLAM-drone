"""
mission_planner.py

Generate a deterministic "lawn-mower" (grid) coverage path inside a mission polygon.

Design goals for this repo:
- No external geometry dependencies (e.g., Shapely) to keep deploy simple.
- Deterministic output for the same inputs (testable).
- Kept minimal; no "optimization" beyond alternating row direction.

Limitations (documented by design):
- Works best for simple, reasonably convex polygons (rectangles are ideal).
- Concave polygons may produce multiple disjoint segments per sweep line; we still
  generate waypoints for each segment, but the transitions between disjoint
  segments may cross outside the polygon.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

import math

PointXY = Tuple[float, float]


@dataclass(frozen=True)
class GridPlan:
    # Ordered path (polyline) in meters.
    path_xy: list[PointXY]
    # For visualization: segments per sweep line (each segment is 2 points).
    grid_segments_xy: list[list[PointXY]]


def _close_polygon(poly: Sequence[PointXY]) -> list[PointXY]:
    if not poly:
        return []
    pts = list(poly)
    if pts[0] != pts[-1]:
        pts.append(pts[0])
    return pts


def _bounds(poly: Sequence[PointXY]) -> Tuple[float, float, float, float]:
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    return (min(xs), min(ys), max(xs), max(ys))


def _line_intersections_with_polygon_y(poly_closed: Sequence[PointXY], y: float) -> list[float]:
    """
    Intersect polygon edges with a horizontal sweep line at Y=y.
    Returns sorted list of X intersection coordinates.
    Uses a half-open rule to avoid double-counting vertices.
    """
    xs: list[float] = []
    for i in range(len(poly_closed) - 1):
        (x1, y1) = poly_closed[i]
        (x2, y2) = poly_closed[i + 1]

        # Skip horizontal edges (either no intersection or infinite).
        if y1 == y2:
            continue

        # Half-open: include y in [min, max) to avoid double-counting vertices.
        if (y1 <= y < y2) or (y2 <= y < y1):
            t = (y - y1) / (y2 - y1)
            x = x1 + t * (x2 - x1)
            if math.isfinite(x):
                xs.append(x)

    xs.sort()
    return xs


def generate_lawnmower_grid(
    polygon_xy: Sequence[PointXY],
    spacing_m: float,
) -> GridPlan:
    """
    Generate a lawn-mower path inside polygon_xy.

    Returns:
      GridPlan(
        path_xy=[(x,y), ...],             # ordered coverage polyline
        grid_segments_xy=[[(x1,y),(x2,y)], ...]  # per-row segments for UI
      )
    """
    if spacing_m <= 0:
        raise ValueError("spacing_m must be > 0")
    if not polygon_xy or len(polygon_xy) < 3:
        raise ValueError("polygon_xy must have at least 3 points")

    poly = _close_polygon(polygon_xy)
    (min_x, min_y, max_x, max_y) = _bounds(poly[:-1])

    # Sweep from south->north (in AirSim terms: -y to +y isn't guaranteed);
    # here we just sweep from min_y to max_y deterministically.
    y = min_y
    row = 0

    path: list[PointXY] = []
    grid_segments: list[list[PointXY]] = []

    # Add a tiny epsilon to include the last line when close to max_y.
    eps = 1e-9
    while y <= max_y + eps:
        xs = _line_intersections_with_polygon_y(poly, y)

        # Pairwise segments (x0,x1), (x2,x3), ...
        segments: list[tuple[float, float]] = []
        for j in range(0, len(xs) - 1, 2):
            x0 = xs[j]
            x1 = xs[j + 1]
            if x1 <= x0:
                continue
            segments.append((x0, x1))

        if segments:
            # Order segments left->right; reverse on odd rows to get a lawn-mower.
            if row % 2 == 1:
                segments = list(reversed(segments))

            for (x0, x1) in segments:
                if row % 2 == 0:
                    a = (x0, y)
                    b = (x1, y)
                else:
                    a = (x1, y)
                    b = (x0, y)

                # Visualization segment always stored left->right for readability.
                grid_segments.append([(min(x0, x1), y), (max(x0, x1), y)])

                # Path appends endpoints (straight line executed by goto).
                if not path:
                    path.append(a)
                    path.append(b)
                else:
                    # Connect to next segment start (may be disjoint for concave polygons).
                    if path[-1] != a:
                        path.append(a)
                    path.append(b)

        row += 1
        y += spacing_m

    if len(path) < 2:
        raise ValueError("mission area too small for the selected grid spacing")

    return GridPlan(path_xy=path, grid_segments_xy=grid_segments)


def to_waypoints_xyz(path_xy: Sequence[PointXY], z_ned: float) -> list[tuple[float, float, float]]:
    return [(x, y, z_ned) for (x, y) in path_xy]


def _point_in_polygon_xy(x: float, y: float, poly: Sequence[PointXY]) -> bool:
    """
    Minimal point-in-polygon for planner validation (boundary-inclusive).
    Used only to avoid generating smoothing points that obviously leave the area.
    """
    if not poly or len(poly) < 3:
        return False
    pts = _close_polygon(poly)

    # Boundary-inclusive check (small epsilon).
    eps = 1e-6
    for i in range(len(pts) - 1):
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        vx = bx - ax
        vy = by - ay
        wx = x - ax
        wy = y - ay
        vv = vx * vx + vy * vy
        if vv <= eps:
            continue
        t = (wx * vx + wy * vy) / vv
        if t < 0.0:
            cx, cy = ax, ay
        elif t > 1.0:
            cx, cy = bx, by
        else:
            cx, cy = ax + t * vx, ay + t * vy
        dx = x - cx
        dy = y - cy
        if (dx * dx + dy * dy) <= (eps * eps):
            return True

    inside = False
    j = len(pts) - 2
    for i in range(len(pts) - 1):
        xi, yi = pts[i]
        xj, yj = pts[j]
        intersects = ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi)
        if intersects:
            inside = not inside
        j = i
    return inside


def smooth_path_turns(
    path_xy: Sequence[PointXY],
    mission_polygon_xy: Sequence[PointXY],
    turn_radius_m: float,
    arc_segments: int,
) -> list[PointXY]:
    """
    Add simple corner fillets (arc points) at sharp turns to reduce sudden yaw changes.

    Important constraints for this repo:
    - Deterministic output (no randomness).
    - No external geometry deps.
    - If smoothing points would leave the mission polygon, we skip smoothing at that turn.
    """
    if turn_radius_m <= 0 or arc_segments <= 0:
        return list(path_xy)
    if len(path_xy) < 3:
        return list(path_xy)

    def norm(vx: float, vy: float) -> tuple[float, float, float]:
        l = math.sqrt(vx * vx + vy * vy)
        if l <= 1e-9:
            return (0.0, 0.0, 0.0)
        return (vx / l, vy / l, l)

    out: list[PointXY] = [path_xy[0]]
    for i in range(1, len(path_xy) - 1):
        prev = path_xy[i - 1]
        cur = path_xy[i]
        nxt = path_xy[i + 1]

        a1x, a1y, len_in = norm(prev[0] - cur[0], prev[1] - cur[1])   # from corner to prev
        a2x, a2y, len_out = norm(nxt[0] - cur[0], nxt[1] - cur[1])    # from corner to next
        if len_in <= 1e-6 or len_out <= 1e-6:
            out.append(cur)
            continue

        # Angle between a1 and a2 (0..pi)
        dot = max(-1.0, min(1.0, a1x * a2x + a1y * a2y))
        phi = math.acos(dot)
        if phi <= math.radians(10.0) or abs(math.pi - phi) <= math.radians(10.0):
            # Too straight (or U-turn-ish); keep original.
            out.append(cur)
            continue

        # Effective radius bounded by segment lengths.
        # Tangency distance t = r * cot(phi/2) must fit on both sides.
        tan_half = math.tan(phi / 2.0) or 1e-9
        r_max_in = len_in * tan_half
        r_max_out = len_out * tan_half
        r = min(turn_radius_m, 0.9 * r_max_in, 0.9 * r_max_out)
        if r <= 1e-3:
            out.append(cur)
            continue

        t = r / tan_half  # r * cot(phi/2)
        p1 = (cur[0] + a1x * t, cur[1] + a1y * t)
        p2 = (cur[0] + a2x * t, cur[1] + a2y * t)

        # Center lies on angle bisector b at distance d = r / sin(phi/2).
        bx, by, _ = norm(a1x + a2x, a1y + a2y)
        sin_half = math.sin(phi / 2.0) or 1e-9
        d = r / sin_half
        center = (cur[0] + bx * d, cur[1] + by * d)

        # Sample arc from p1 -> p2 around center.
        a0 = math.atan2(p1[1] - center[1], p1[0] - center[0])
        a1 = math.atan2(p2[1] - center[1], p2[0] - center[0])

        # Choose the shorter rotation direction.
        da = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi

        arc_pts: list[PointXY] = [p1]
        for k in range(1, arc_segments):
            ak = a0 + da * (k / arc_segments)
            arc_pts.append((center[0] + r * math.cos(ak), center[1] + r * math.sin(ak)))
        arc_pts.append(p2)

        # Validate arc points are within polygon (otherwise keep the sharp corner).
        if all(_point_in_polygon_xy(px, py, mission_polygon_xy) for (px, py) in arc_pts):
            # Replace the corner with the arc; avoid duplicates.
            if out[-1] != arc_pts[0]:
                out.append(arc_pts[0])
            out.extend(arc_pts[1:])
        else:
            out.append(cur)

    out.append(path_xy[-1])
    return out
