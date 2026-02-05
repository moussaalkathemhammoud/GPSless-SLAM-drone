"""
slam_airsim_alignment.py

Additive helper to translate SLAM-frame coordinates (ORB-SLAM) to AirSim NED XY.

Why this exists:
- The SLAM keyframe map you draw in the UI is in SLAM coordinates (typically X/Z on ground plane).
- AirSim goto expects AirSim local NED meters (x,y on ground plane).
- Even with stereo (metric scale), SLAM's frame can be rotated/translated relative to AirSim.

We solve a 2D rigid (or similarity) transform using paired samples:
  slam (sx, sz)  ->  airsim (x, y)

This does NOT modify SLAM internals; it only uses SLAM outputs already available and AirSim pose.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Iterable, Literal


BASE_DIR = Path(__file__).resolve().parent.parent  # slam_web/
PAIRS_FILE = BASE_DIR / "missions" / "slam_airsim_pairs.json"
TRANSFORM_FILE = BASE_DIR / "missions" / "slam_to_airsim_transform.json"


@dataclass(frozen=True)
class Pair:
    sx: float
    sy: float
    sz: float
    ax: float
    ay: float
    # Optional metadata for calibration diagnostics / recomputation.
    ax_body: float | None = None
    ay_body: float | None = None
    yaw_deg: float | None = None


def _load_json(path: Path) -> dict:
    try:
        if not path.exists():
            return {}
        data = json.loads(path.read_text())
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def load_pairs() -> list[Pair]:
    data = _load_json(PAIRS_FILE)
    raw = data.get("pairs", [])
    out: list[Pair] = []
    if not isinstance(raw, list):
        return out
    for p in raw:
        try:
            # Backwards compatible: older files may not contain sy.
            ax_body = p.get("ax_body")
            ay_body = p.get("ay_body")
            yaw_deg = p.get("yaw_deg")
            out.append(
                Pair(
                    sx=float(p["sx"]),
                    sy=float(p.get("sy", 0.0)),
                    sz=float(p["sz"]),
                    ax=float(p["ax"]),
                    ay=float(p["ay"]),
                    ax_body=None if ax_body is None else float(ax_body),
                    ay_body=None if ay_body is None else float(ay_body),
                    yaw_deg=None if yaw_deg is None else float(yaw_deg),
                )
            )
        except Exception:
            continue
    return out


def save_pairs(pairs: Iterable[Pair]) -> None:
    PAIRS_FILE.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "pairs": [],
    }
    for p in pairs:
        d = {"sx": p.sx, "sy": p.sy, "sz": p.sz, "ax": p.ax, "ay": p.ay}
        if p.ax_body is not None:
            d["ax_body"] = p.ax_body
        if p.ay_body is not None:
            d["ay_body"] = p.ay_body
        if p.yaw_deg is not None:
            d["yaw_deg"] = p.yaw_deg
        payload["pairs"].append(d)
    PAIRS_FILE.write_text(json.dumps(payload, indent=2))


def reset_pairs() -> None:
    save_pairs([])


def append_pair(pair: Pair) -> list[Pair]:
    pairs = load_pairs()
    pairs.append(pair)
    save_pairs(pairs)
    return pairs


@dataclass(frozen=True)
class Transform2D:
    # Maps SLAM ground plane (sx,sz) -> AirSim ground plane (x,y)
    scale: float
    r11: float
    r12: float
    r21: float
    r22: float
    tx: float
    ty: float
    model: Literal["rigid", "similarity"]
    slam_axes: tuple[Literal["x", "y", "z"], Literal["x", "y", "z"]]

    def apply_uv(self, u: float, v: float) -> tuple[float, float]:
        x = self.scale * (self.r11 * u + self.r12 * v) + self.tx
        y = self.scale * (self.r21 * u + self.r22 * v) + self.ty
        return (x, y)

    def apply_xyz(self, sx: float, sy: float, sz: float) -> tuple[float, float]:
        u = sx if self.slam_axes[0] == "x" else (sy if self.slam_axes[0] == "y" else sz)
        v = sx if self.slam_axes[1] == "x" else (sy if self.slam_axes[1] == "y" else sz)
        return self.apply_uv(u, v)


def _mean(xs: list[float]) -> float:
    return sum(xs) / max(1, len(xs))


def solve_transform(pairs: list[Pair], allow_scale: bool, slam_axes: tuple[Literal["x", "y", "z"], Literal["x", "y", "z"]]) -> Transform2D:
    """
    Least-squares Umeyama-style solve in 2D.

    With stereo SLAM, allow_scale=False is usually correct (scale≈1).
    """
    if len(pairs) < 2:
        raise ValueError("Need at least 2 pairs to solve a 2D transform")

    def _u(p: Pair) -> float:
        return p.sx if slam_axes[0] == "x" else (p.sy if slam_axes[0] == "y" else p.sz)

    def _v(p: Pair) -> float:
        return p.sx if slam_axes[1] == "x" else (p.sy if slam_axes[1] == "y" else p.sz)

    su = [_u(p) for p in pairs]
    sv = [_v(p) for p in pairs]
    ax = [p.ax for p in pairs]
    ay = [p.ay for p in pairs]

    msu = _mean(su)
    msv = _mean(sv)
    maxx = _mean(ax)
    mayy = _mean(ay)

    # Centered coordinates
    xs = [(_u(p) - msu, _v(p) - msv) for p in pairs]
    ya = [(p.ax - maxx, p.ay - mayy) for p in pairs]

    # Cross-covariance H = X^T Y (2x2)
    h11 = sum(x[0] * y[0] for x, y in zip(xs, ya))
    h12 = sum(x[0] * y[1] for x, y in zip(xs, ya))
    h21 = sum(x[1] * y[0] for x, y in zip(xs, ya))
    h22 = sum(x[1] * y[1] for x, y in zip(xs, ya))

    # For 2D, we can get the best rotation from H using a closed-form:
    # R = [ c -s; s c ] where:
    #   c = (h11 + h22) / sqrt((h11 + h22)^2 + (h21 - h12)^2)
    #   s = (h21 - h12) / sqrt((h11 + h22)^2 + (h21 - h12)^2)
    a = (h11 + h22)
    b = (h21 - h12)
    denom = math.sqrt(a * a + b * b)
    if denom <= 1e-12:
        raise ValueError("Degenerate calibration (points collinear or identical); move the drone to more distinct positions")
    c = a / denom
    s = b / denom

    r11, r12 = c, -s
    r21, r22 = s, c

    scale = 1.0
    model: Literal["rigid", "similarity"] = "rigid"
    if allow_scale:
        # scale = trace(S) / var(X) where S is singular values (in 2D closed-form):
        # Here denom approximates sum singular values for 2x2 H after rotation solve.
        var_x = sum(x[0] * x[0] + x[1] * x[1] for x in xs)
        if var_x > 1e-12:
            scale = denom / var_x
            model = "similarity"

    # Translation t = mu_y - s*R*mu_x
    tx = maxx - scale * (r11 * msu + r12 * msv)
    ty = mayy - scale * (r21 * msu + r22 * msv)

    return Transform2D(
        scale=scale,
        r11=r11,
        r12=r12,
        r21=r21,
        r22=r22,
        tx=tx,
        ty=ty,
        model=model,
        slam_axes=slam_axes,
    )


def solve_best_transform(pairs: list[Pair], allow_scale: bool) -> Transform2D:
    """
    Try all plausible SLAM ground-plane axis pairings and choose the one with the lowest RMSE.
    This handles cases where the upstream SLAM sender uses (x,y) instead of (x,z) for the ground plane.
    """
    axes = [("x", "y"), ("x", "z"), ("y", "z")]
    best: Transform2D | None = None
    best_rmse = float("inf")
    for a in axes:
        try:
            t = solve_transform(pairs, allow_scale=allow_scale, slam_axes=a)  # type: ignore[arg-type]
            e = rmse_m(t, pairs)
            if e < best_rmse:
                best_rmse = e
                best = t
        except Exception:
            continue
    if best is None:
        raise ValueError("Failed to solve transform for any axis pairing (need more diverse motion)")
    return best


def save_transform(
    t: Transform2D,
    pairs_used: int,
    *,
    rmse_m: float | None = None,
    quality: str | None = None,
    ts: float | None = None,
) -> None:
    TRANSFORM_FILE.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "model": t.model,
        "scale": t.scale,
        "R": [[t.r11, t.r12], [t.r21, t.r22]],
        "t": [t.tx, t.ty],
        "slam_axes": [t.slam_axes[0], t.slam_axes[1]],
        "pairs_used": pairs_used,
    }
    if rmse_m is not None:
        payload["rmse_m"] = float(rmse_m)
    if quality is not None:
        payload["quality"] = str(quality)
    if ts is not None:
        payload["ts"] = float(ts)
    TRANSFORM_FILE.write_text(json.dumps(payload, indent=2))


def load_transform_payload() -> dict:
    """
    Returns the raw JSON payload stored on disk for the current transform.
    This lets callers read metadata like rmse/quality without changing Transform2D.
    """
    return _load_json(TRANSFORM_FILE)


def reset_transform() -> None:
    """
    Remove the saved transform from disk (pairs are kept separately).
    Used when SLAM restarts so we don't apply a stale transform to a new SLAM world frame.
    """
    try:
        if TRANSFORM_FILE.exists():
            TRANSFORM_FILE.unlink()
    except Exception:
        pass


def load_transform() -> Transform2D | None:
    data = _load_json(TRANSFORM_FILE)
    try:
        r = data["R"]
        tt = data["t"]
        model = data.get("model", "rigid")
        axes = data.get("slam_axes", ["x", "z"])
        if not (isinstance(axes, list) and len(axes) == 2 and axes[0] in ("x", "y", "z") and axes[1] in ("x", "y", "z")):
            axes = ["x", "z"]
        return Transform2D(
            model="similarity" if model == "similarity" else "rigid",
            scale=float(data.get("scale", 1.0)),
            r11=float(r[0][0]),
            r12=float(r[0][1]),
            r21=float(r[1][0]),
            r22=float(r[1][1]),
            tx=float(tt[0]),
            ty=float(tt[1]),
            slam_axes=(axes[0], axes[1]),  # type: ignore[arg-type]
        )
    except Exception:
        return None


def rmse_m(t: Transform2D, pairs: list[Pair]) -> float:
    if not pairs:
        return 0.0
    err2 = 0.0
    for p in pairs:
        x, y = t.apply_xyz(p.sx, p.sy, p.sz)
        dx = x - p.ax
        dy = y - p.ay
        err2 += dx * dx + dy * dy
    return math.sqrt(err2 / len(pairs))


def residuals_m(t: Transform2D, pairs: list[Pair]) -> list[float]:
    """
    Per-pair 2D residual magnitude in meters.
    Used for robust trimming / outlier rejection.
    """
    out: list[float] = []
    for p in pairs:
        x, y = t.apply_xyz(p.sx, p.sy, p.sz)
        dx = x - p.ax
        dy = y - p.ay
        out.append(float((dx * dx + dy * dy) ** 0.5))
    return out


def solve_transform_robust(
    pairs: list[Pair],
    *,
    allow_scale: bool = True,
    keep_fraction: float = 0.7,
    min_keep: int = 4,
    axes_candidates: list[tuple[Literal["x", "y", "z"], Literal["x", "y", "z"]]] | None = None,
) -> tuple[Transform2D, float, int]:
    """
    Robust solve wrapper around solve_transform():
    - Tries multiple SLAM ground-plane axis candidates
    - Solves once, trims worst residuals, re-solves
    - Returns (transform, rmse_m, pairs_used)
    """
    if len(pairs) < 4:
        raise ValueError("need >=4 pairs")

    if axes_candidates is None:
        axes_candidates = [("x", "z"), ("x", "y"), ("y", "z")]

    keep_fraction = float(keep_fraction)
    if not (0.2 <= keep_fraction <= 1.0):
        keep_fraction = 0.7
    min_keep = max(4, int(min_keep))

    best_t: Transform2D | None = None
    best_rmse = float("inf")
    best_used = 0

    for axes in axes_candidates:
        try:
            t0 = solve_transform(pairs, allow_scale=bool(allow_scale), slam_axes=axes)
            res0 = residuals_m(t0, pairs)
            idx = sorted(range(len(pairs)), key=lambda i: res0[i])

            keep = max(min_keep, int(keep_fraction * len(pairs)))
            keep = min(keep, len(pairs))
            trimmed = [pairs[i] for i in idx[:keep]]

            t1 = solve_transform(trimmed, allow_scale=bool(allow_scale), slam_axes=axes)
            rmse = rmse_m(t1, trimmed)

            if rmse < best_rmse:
                best_rmse = float(rmse)
                best_t = t1
                best_used = len(trimmed)
        except Exception:
            continue

    if best_t is None:
        raise ValueError("robust solve failed")

    return (best_t, float(best_rmse), int(best_used))
