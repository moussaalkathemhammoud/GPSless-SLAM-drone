from __future__ import annotations

import json
import time
import uuid
from pathlib import Path
from typing import Any, List, Optional

import anyio
import asyncio
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from .drone_control import send_mission_abort, send_mission_start, send_drone_mode
from .drone_control import send_goto
from .drone_process import start_drone
from .slam_control import send_slam_mode
from .slam_session import get_slam_state, set_slam_state
from .gps_utils import (
    FakeGpsOrigin,
    airsim_xy_to_latlng,
    get_fake_gps_offset_xy_m,
    get_fake_gps_origin,
    get_fake_gps_yaw_deg,
    latlng_to_airsim_xy,
)
from .mission_planner import generate_lawnmower_grid, smooth_path_turns, to_waypoints_xyz
from .airsim_pose import get_airsim_pose
from .mission_pair_recorder import start_mission_pair_recording, stop_mission_pair_recording


router = APIRouter(prefix="/mission", tags=["Mission"])

BASE_DIR = Path(__file__).resolve().parent.parent  # slam_web/
MISSIONS_DIR = BASE_DIR / "missions"
CALIB_FILE = BASE_DIR / "calibration.json"
POSE_FILE = MISSIONS_DIR / "airsim_pose.json"


class LatLng(BaseModel):
    lat: float
    lng: float


class MissionCreateReq(BaseModel):
    # Leaflet polygon/rectangle: array of vertices in order.
    mission_area: list[LatLng] = Field(..., description="List of polygon vertices (lat/lng).")
    # Mission altitude in meters above ground (positive). Converted to NED z (negative is up).
    altitude: float = Field(..., gt=0.1)


class MissionCreateResp(BaseModel):
    ok: bool
    mission_id: str
    origin: LatLng
    yaw_deg: float
    offset_x_m: float
    offset_y_m: float
    altitude_m: float
    grid_spacing_m: float
    # Echo for UI
    mission_area: list[LatLng]
    # Visualization-only helpers (no SLAM math): precomputed grid overlays.
    grid_segments: list[list[LatLng]]
    path: list[LatLng]


class MissionStartReq(BaseModel):
    mission_id: Optional[str] = None


class MissionAbortReq(BaseModel):
    mission_id: Optional[str] = None


class MissionGotoReq(BaseModel):
    lat: float
    lng: float
    altitude_m: float = Field(10.0, gt=0.1, description="Meters AGL (converted to NED z<0).")


class CalibSnapReq(BaseModel):
    lat: float
    lng: float


class CalibSetOriginReq(BaseModel):
    lat: float
    lng: float
    # Optional: move the drone to the new origin at this altitude (meters AGL).
    # If omitted, we keep current z (if available) or default to 10m AGL.
    altitude_m: Optional[float] = None


_latest_mission_id: str | None = None
_latest_mission_file: Path | None = None


class MissionLastResp(BaseModel):
    ok: bool
    mission_id: str | None = None
    mission_area: list[LatLng] | None = None
    origin: LatLng | None = None
    yaw_deg: float | None = None
    offset_x_m: float | None = None
    offset_y_m: float | None = None


def _grid_spacing_m() -> float:
    # Configurable, but kept server-side for determinism and simple UI.
    import os

    try:
        v = float(os.getenv("MISSION_GRID_SPACING_M", "5.0"))
    except Exception:
        v = 5.0
    return max(0.5, v)

def _turn_radius_m(spacing_m: float) -> float:
    import os

    # Conservative default: small fillet that still helps SLAM during row turns.
    try:
        v = float(os.getenv("MISSION_TURN_RADIUS_M", "2.0"))
    except Exception:
        v = 2.0
    # Avoid generating arcs larger than the row spacing.
    return max(0.0, min(v, max(0.0, spacing_m * 0.45)))


def _turn_arc_segments() -> int:
    import os

    try:
        v = int(os.getenv("MISSION_TURN_ARC_SEGMENTS", "5"))
    except Exception:
        v = 5
    return max(0, min(v, 20))


def _ensure_missions_dir() -> None:
    try:
        MISSIONS_DIR.mkdir(parents=True, exist_ok=True)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to create missions dir: {e}")


def _write_mission_file(payload: dict[str, Any]) -> Path:
    _ensure_missions_dir()
    mission_id = payload["mission_id"]
    path = MISSIONS_DIR / f"{mission_id}.json"
    try:
        path.write_text(json.dumps(payload, indent=2))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to write mission file: {e}")
    return path


def _is_mission_id(s: str) -> bool:
    # Mission ids are uuid4 hex (32 chars).
    if len(s) != 32:
        return False
    try:
        int(s, 16)
        return True
    except Exception:
        return False


def _find_latest_mission_file() -> Path | None:
    _ensure_missions_dir()
    candidates: list[Path] = []
    for p in MISSIONS_DIR.glob("*.json"):
        name = p.stem
        if not _is_mission_id(name):
            continue
        if name == "slam_airsim_pairs" or name == "airsim_pose" or name == "slam_to_airsim_transform":
            continue
        candidates.append(p)
    if not candidates:
        return None
    candidates.sort(key=lambda x: x.stat().st_mtime, reverse=True)
    return candidates[0]


@router.get("/last", response_model=MissionLastResp)
def mission_last() -> MissionLastResp:
    """
    Returns the most recently created mission polygon (lat/lng) plus the current
    geo origin/yaw/offset used for conversions.

    This is a UI helper so the frontend can rebuild overlays after refresh/restart.
    """
    global _latest_mission_id, _latest_mission_file

    mission_file = _latest_mission_file
    if not mission_file or not mission_file.exists():
        mission_file = _find_latest_mission_file()

    if not mission_file or not mission_file.exists():
        return MissionLastResp(ok=True, mission_id=None, mission_area=None)

    try:
        payload = json.loads(mission_file.read_text())
    except Exception:
        return MissionLastResp(ok=True, mission_id=None, mission_area=None)

    mission_id = str(payload.get("mission_id") or mission_file.stem)
    raw_area = payload.get("mission_area")
    if not isinstance(raw_area, list) or len(raw_area) < 3:
        return MissionLastResp(ok=True, mission_id=mission_id, mission_area=None)

    area: list[LatLng] = []
    for p in raw_area:
        if not isinstance(p, dict):
            continue
        try:
            area.append(LatLng(lat=float(p["lat"]), lng=float(p["lng"])))
        except Exception:
            continue

    origin = get_fake_gps_origin()
    yaw_deg = get_fake_gps_yaw_deg()
    offset_xy = get_fake_gps_offset_xy_m()

    _latest_mission_id = mission_id
    _latest_mission_file = mission_file

    return MissionLastResp(
        ok=True,
        mission_id=mission_id,
        mission_area=area if len(area) >= 3 else None,
        origin=LatLng(lat=origin.lat0, lng=origin.lng0),
        yaw_deg=float(yaw_deg),
        offset_x_m=float(offset_xy[0]),
        offset_y_m=float(offset_xy[1]),
    )


@router.post("/create", response_model=MissionCreateResp)
def mission_create(req: MissionCreateReq):
    """
    Creates a mission (does not start motion).
    - Converts fake GPS polygon -> AirSim XY (meters)
    - Generates a lawn-mower grid path inside it
    - Writes a mission JSON file for the long-running drone_motion process
      (avoids UDP packet size limits).
    """
    global _latest_mission_id, _latest_mission_file

    if len(req.mission_area) < 3:
        raise HTTPException(status_code=400, detail="mission_area must have at least 3 vertices")

    origin = get_fake_gps_origin()
    yaw_deg = get_fake_gps_yaw_deg()
    offset_xy = get_fake_gps_offset_xy_m()
    poly_xy = [latlng_to_airsim_xy(p.lat, p.lng, origin, yaw_deg=yaw_deg, offset_xy_m=offset_xy) for p in req.mission_area]

    spacing = _grid_spacing_m()
    try:
        grid = generate_lawnmower_grid(poly_xy, spacing_m=spacing)
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))

    altitude_m = float(req.altitude)
    z_ned = -abs(altitude_m)

    mission_id = uuid.uuid4().hex
    created_at = time.time()

    # Smooth turns by inserting arc points at sharp corners (still executed via goto).
    turn_r = _turn_radius_m(spacing)
    turn_seg = _turn_arc_segments()
    path_xy = smooth_path_turns(grid.path_xy, poly_xy, turn_radius_m=turn_r, arc_segments=turn_seg)
    waypoints_xyz = to_waypoints_xyz(path_xy, z_ned=z_ned)

    payload = {
        "mission_id": mission_id,
        "created_at": created_at,
        "origin": {"lat": origin.lat0, "lng": origin.lng0},
        "altitude_m": altitude_m,
        "z_ned": z_ned,
        # Operator intent: keep mapping/coverage running until explicitly aborted/stopped.
        # drone_motion will loop waypoints when loop=true.
        "loop": True,
        "grid_spacing_m": spacing,
        "mission_area": [{"lat": p.lat, "lng": p.lng} for p in req.mission_area],
        "mission_area_xy": [[x, y] for (x, y) in poly_xy],
        "grid_segments_xy": [[[a[0], a[1]], [b[0], b[1]]] for (a, b) in grid.grid_segments_xy],
        "path_xy": [[x, y] for (x, y) in path_xy],
        "waypoints_xyz": [[x, y, z] for (x, y, z) in waypoints_xyz],
    }

    mission_file = _write_mission_file(payload)
    _latest_mission_id = mission_id
    _latest_mission_file = mission_file

    # Convert overlays back to lat/lng for Leaflet.
    def _xy_to_latlng(x: float, y: float) -> LatLng:
        lat, lng = airsim_xy_to_latlng(x, y, origin, yaw_deg=yaw_deg, offset_xy_m=offset_xy)
        return LatLng(lat=lat, lng=lng)

    grid_segments_latlng = [
        [_xy_to_latlng(seg[0][0], seg[0][1]), _xy_to_latlng(seg[1][0], seg[1][1])]
        for seg in grid.grid_segments_xy
    ]
    path_latlng = [_xy_to_latlng(x, y) for (x, y) in path_xy]

    return MissionCreateResp(
        ok=True,
        mission_id=mission_id,
        origin=LatLng(lat=origin.lat0, lng=origin.lng0),
        yaw_deg=yaw_deg,
        offset_x_m=float(offset_xy[0]),
        offset_y_m=float(offset_xy[1]),
        altitude_m=altitude_m,
        grid_spacing_m=spacing,
        mission_area=req.mission_area,
        grid_segments=grid_segments_latlng,
        path=path_latlng,
    )


@router.post("/start")
def mission_start(req: MissionStartReq):
    global _latest_mission_id, _latest_mission_file
    if get_slam_state() == "calibrating":
        raise HTTPException(status_code=409, detail="Calibration is running. Wait for it to finish.")

    mission_id = req.mission_id or _latest_mission_id
    if not mission_id:
        raise HTTPException(status_code=400, detail="No mission available. Call /mission/create first.")

    mission_file = (MISSIONS_DIR / f"{mission_id}.json")
    if not mission_file.exists():
        raise HTTPException(status_code=400, detail="No mission available. Call /mission/create first.")

    # Ensure the long-running drone_motion process exists.
    # (Do NOT kill it here; start_drone() is idempotent.)
    start_drone()

    # Keep SLAM in mapping mode, but put drone motion in explore so static mapping
    # does not fight the mission gotos.
    send_slam_mode("mapping")
    set_slam_state("mapping")
    send_drone_mode("explore")
    # Always record alignment pairs while the mission runs.
    try:
        start_mission_pair_recording()
    except Exception:
        pass

    try:
        payload = json.loads(mission_file.read_text())
        waypoints_count = len(payload.get("waypoints_xyz", []) or [])
    except Exception:
        waypoints_count = None

    send_mission_start(mission_id=mission_id, mission_file=str(mission_file))
    return {"ok": True, "mission_id": mission_id, "mission_file": str(mission_file), "waypoints": waypoints_count}


@router.post("/abort")
def mission_abort(req: MissionAbortReq):
    mission_id = req.mission_id or _latest_mission_id
    if not mission_id:
        raise HTTPException(status_code=400, detail="No mission_id provided and no active mission.")

    send_mission_abort(mission_id=mission_id)
    try:
        stop_mission_pair_recording()
    except Exception:
        pass
    return {"ok": True, "mission_id": mission_id}


@router.post("/goto")
def mission_goto(req: MissionGotoReq):
    """
    Geo-map click-to-go helper (fake GPS -> AirSim).

    This exists because the SLAM keyframe map is in SLAM coordinates, which may not match
    AirSim NED. Geo map lat/lng is always aligned to the mission/mapping coordinate system.
    """
    origin = get_fake_gps_origin()
    yaw_deg = get_fake_gps_yaw_deg()
    offset_xy = get_fake_gps_offset_xy_m()

    x, y = latlng_to_airsim_xy(req.lat, req.lng, origin, yaw_deg=yaw_deg, offset_xy_m=offset_xy)
    z_ned = -abs(float(req.altitude_m))

    start_drone()
    if get_slam_state() == "calibrating":
        raise HTTPException(status_code=409, detail="Calibration is running. Wait for it to finish.")
    # Ensure a running mission doesn't keep overriding the operator's manual goto.
    try:
        send_mission_abort(mission_id="")
    except Exception:
        pass
    send_slam_mode("localization")
    set_slam_state("localization")
    send_drone_mode("explore")
    send_goto(float(x), float(y), float(z_ned))
    return {"ok": True, "target": {"x": float(x), "y": float(y), "z": float(z_ned)}}


@router.get("/drone/pose")
async def drone_pose_geo():
    """
    Geo-referenced drone pose for the image-based "GPS" map.
    Returns BOTH AirSim meters and fake-GPS lat/lng (frontend should use lat/lng).
    """
    origin = get_fake_gps_origin()
    yaw_deg = get_fake_gps_yaw_deg()
    offset_xy = get_fake_gps_offset_xy_m()

    # Prefer pose published by drone_motion process (avoids AirSim RPC in FastAPI).
    pose: dict[str, Any] | None = None
    try:
        if POSE_FILE.exists():
            pose = json.loads(POSE_FILE.read_text())
    except Exception:
        pose = None

    if pose is None:
        try:
            # Fallback: run AirSim RPC in a worker thread.
            pose = await anyio.to_thread.run_sync(get_airsim_pose)
        except asyncio.CancelledError:
            raise HTTPException(status_code=503, detail="Shutting down")
        except Exception as e:
            raise HTTPException(status_code=503, detail=f"AirSim not reachable: {type(e).__name__}: {e}")

    lat, lng = airsim_xy_to_latlng(float(pose["x"]), float(pose["y"]), origin, yaw_deg=yaw_deg, offset_xy_m=offset_xy)
    return {
        "ok": True,
        "origin": {"lat": origin.lat0, "lng": origin.lng0},
        "yaw_deg": yaw_deg,
        "offset_x_m": offset_xy[0],
        "offset_y_m": offset_xy[1],
        "x": float(pose["x"]),
        "y": float(pose["y"]),
        "z": float(pose.get("z", 0.0)),
        "yaw_drone_deg": float(pose.get("yaw_deg", pose.get("yaw", 0.0))),
        "lat": lat,
        "lng": lng,
        "has_collided": bool(pose.get("has_collided", False)),
        "collision_time_stamp": float(pose.get("collision_time_stamp", 0.0)),
    }


@router.get("/calibration")
def get_calibration():
    origin = get_fake_gps_origin()
    yaw_deg = get_fake_gps_yaw_deg()
    offset_xy = get_fake_gps_offset_xy_m()
    return {
        "ok": True,
        "lat0": origin.lat0,
        "lng0": origin.lng0,
        "yaw_deg": yaw_deg,
        "offset_x_m": offset_xy[0],
        "offset_y_m": offset_xy[1],
        "file": str(CALIB_FILE),
    }


@router.post("/calibration/snap")
async def calibration_snap(req: CalibSnapReq):
    """
    Calibration helper:
    - User clicks the *true* drone position on the image map (lat/lng in our fake GPS).
    - Backend reads current drone AirSim pose (x,y).
    - We compute FAKE_GPS_OFFSET_X/Y so that the current drone pose maps exactly to that click.
    """
    origin = get_fake_gps_origin()
    yaw_deg = get_fake_gps_yaw_deg()

    try:
        pose = await anyio.to_thread.run_sync(get_airsim_pose)
    except asyncio.CancelledError:
        raise HTTPException(status_code=503, detail="Shutting down")
    except Exception as e:
        raise HTTPException(status_code=503, detail=f"AirSim not reachable: {type(e).__name__}: {e}")

    # Compute where the clicked lat/lng would be in AirSim if offset=(0,0).
    x_no_off, y_no_off = latlng_to_airsim_xy(req.lat, req.lng, origin, yaw_deg=yaw_deg, offset_xy_m=(0.0, 0.0))
    offset_x = float(pose["x"]) - float(x_no_off)
    offset_y = float(pose["y"]) - float(y_no_off)

    payload = {
        "lat0": origin.lat0,
        "lng0": origin.lng0,
        "yaw_deg": yaw_deg,
        "offset_x_m": offset_x,
        "offset_y_m": offset_y,
    }
    try:
        CALIB_FILE.write_text(json.dumps(payload, indent=2))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to write calibration file: {e}")

    return {"ok": True, **payload, "file": str(CALIB_FILE)}


@router.post("/calibration/set_origin")
def calibration_set_origin(req: CalibSetOriginReq):
    """
    Convenience calibration:
    - User clicks a point on the image map.
    - We declare that point to be the fake GPS origin (AirSim x=0,y=0).

    This effectively "moves the map under the drone" without changing the AirSim spawn.
    """
    yaw_deg = get_fake_gps_yaw_deg()
    payload = {
        "lat0": float(req.lat),
        "lng0": float(req.lng),
        "yaw_deg": float(yaw_deg),
        "offset_x_m": 0.0,
        "offset_y_m": 0.0,
    }
    try:
        CALIB_FILE.write_text(json.dumps(payload, indent=2))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to write calibration file: {e}")

    return {"ok": True, **payload, "file": str(CALIB_FILE)}


@router.post("/calibration/origin_at_drone")
async def calibration_origin_at_drone():
    """
    Sets the fake GPS origin to the drone's CURRENT AirSim XY pose (without moving the drone).

    Practically:
      - Keeps lat0/lng0 unchanged (map anchor stays the same).
      - Sets offset_x_m/offset_y_m = current drone (x,y), so that the drone appears at origin on the map.

    Use this when the drone spawns at a non-zero AirSim coordinate but you want that spawn to be treated
    as the "origin" for missions and visualization.
    """
    origin = get_fake_gps_origin()
    yaw_deg = get_fake_gps_yaw_deg()
    try:
        pose = await anyio.to_thread.run_sync(get_airsim_pose)
    except asyncio.CancelledError:
        raise HTTPException(status_code=503, detail="Shutting down")
    except Exception as e:
        raise HTTPException(status_code=503, detail=f"AirSim not reachable: {type(e).__name__}: {e}")

    payload = {
        "lat0": float(origin.lat0),
        "lng0": float(origin.lng0),
        "yaw_deg": float(yaw_deg),
        "offset_x_m": float(pose["x"]),
        "offset_y_m": float(pose["y"]),
    }
    try:
        CALIB_FILE.write_text(json.dumps(payload, indent=2))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to write calibration file: {e}")

    return {"ok": True, **payload, "file": str(CALIB_FILE)}


@router.post("/calibration/set_origin_and_move")
async def calibration_set_origin_and_move(req: CalibSetOriginReq):
    """
    Calibration helper (recommended):
    - User clicks a point on the map where they want the origin to be.
    - We compute the corresponding AirSim (x,y) using the *current* calibration.
    - We command the drone to fly there (goto), and then we write calibration.json so
      that the clicked point becomes the origin AND the drone's target pose equals that origin.

    NOTE:
    This physically moves the drone. Use with care (ensure you're in a safe altitude).
    """
    origin = get_fake_gps_origin()
    yaw_deg = get_fake_gps_yaw_deg()
    offset_xy = get_fake_gps_offset_xy_m()

    # Compute the AirSim target for the clicked map point under current calibration.
    x_target, y_target = latlng_to_airsim_xy(req.lat, req.lng, origin, yaw_deg=yaw_deg, offset_xy_m=offset_xy)

    # Choose altitude for goto.
    try:
        pose = await anyio.to_thread.run_sync(get_airsim_pose)
        z_cur = float(pose.get("z", -10.0))
    except Exception:
        z_cur = -10.0

    if req.altitude_m is not None:
        z_ned = -abs(float(req.altitude_m))
    else:
        # Keep current altitude (locked altitude requirement); fall back to -10m.
        z_ned = z_cur if z_cur != 0.0 else -10.0

    # Ensure drone_motion is running and in explore mode for goto.
    start_drone()
    send_drone_mode("explore")
    from .drone_control import send_goto

    send_goto(float(x_target), float(y_target), float(z_ned))

    # Persist calibration such that the clicked point is the origin AND that origin corresponds
    # to the target AirSim pose (offset = target).
    payload = {
        "lat0": float(req.lat),
        "lng0": float(req.lng),
        "yaw_deg": float(yaw_deg),
        "offset_x_m": float(x_target),
        "offset_y_m": float(y_target),
    }
    try:
        CALIB_FILE.write_text(json.dumps(payload, indent=2))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to write calibration file: {e}")

    return {
        "ok": True,
        **payload,
        "file": str(CALIB_FILE),
        "airsim_target": {"x": float(x_target), "y": float(y_target), "z": float(z_ned)},
    }
