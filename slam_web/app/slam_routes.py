# app/slam_routes.py
from fastapi import APIRouter, HTTPException
from fastapi.responses import JSONResponse
from pydantic import BaseModel
from .slam_process import start_slam, stop_slam, slam_status
from .udp_listener import latest_pose, get_trajectory_segments, get_udp_stats
from .drone_process import start_drone, stop_drone
from .kf_reader import KF_FILE, read_keyframes
from .drone_control import (
    send_calib_run,
    send_calib_stop,
    send_calib_static_run,
    send_forward,
    send_goto,
    send_hover,
    send_drone_mode,
    send_mission_abort,
)
from .slam_control import send_slam_mode   # ✅ NEW (controls ORB-SLAM bridge)
from .slam_session import get_slam_state, set_slam_state
from .mission_pair_recorder import stop_mission_pair_recording
from .shared_paths import AIRSIM_POSE_FILE as _AIRSIM_POSE_FILE
from .slam_airsim_alignment import (
    Pair,
    append_pair,
    load_pairs,
    load_transform_payload,
    load_transform,
    reset_pairs,
    reset_transform,
    rmse_m,
    save_pairs,
    save_transform,
    solve_transform,
    solve_best_transform,
    solve_transform_robust,
)

import json
import math
import os
import threading
import time
from pathlib import Path
from typing import Literal

router = APIRouter(prefix="/slam", tags=["SLAM"])

# Written by drone_motion during calibration-only motion.
_CALIB_MOTION_STATUS_FILE = (Path(__file__).resolve().parent.parent / "missions" / "calib_motion_status.json")

def _calibration_requires_localization() -> JSONResponse:
    return JSONResponse(
        status_code=400,
        content={"ok": False, "error": "Calibration requires SLAM localization mode. Finalize map first."},
    )


def _rmse_quality(rmse: float | None) -> str | None:
    """
    Human-readable quality tier for SLAM->AirSim alignment.
    RMSE is in meters on the ground plane.
    """
    if rmse is None:
        return None
    if rmse > 4.0:
        return "poor"
    if rmse > 2.0:
        return "ok"
    if rmse > 1.0:
        return "good"
    return "great"


def _rmse_goto_limit_m(rmse: float | None) -> float | None:
    """
    For SLAM-based click-to-go, use RMSE to limit authority (not existence).
    Returns max allowed step distance in meters, or None for no limit.
    """
    if rmse is None:
        return 0.0
    if rmse > 4.0:
        return 0.0  # disabled
    if rmse > 2.0:
        return 5.0
    if rmse > 1.0:
        return 20.0
    return None


def _slam_axes_locked() -> tuple[str, str]:
    """
    Axis choice is a semantic decision: for ORB-SLAM style outputs, ground plane is usually (x,z).
    We keep this locked for stability; auto axis switching makes incremental solving unstable.
    """
    import os

    raw = os.getenv("SLAM_ALIGN_AXES", "x,z").strip().lower().replace(" ", "")
    if raw in ("auto", "best"):
        return ("auto", "auto")
    a, b = "x", "z"
    try:
        parts = [p for p in raw.split(",") if p]
        if len(parts) >= 2:
            a, b = parts[0], parts[1]
    except Exception:
        pass
    if a not in ("x", "y", "z") or b not in ("x", "y", "z") or a == b:
        a, b = "x", "z"
    return (a, b)


def _solve_transform_from_pairs(pairs: list[Pair], *, allow_scale: bool):
    axes = _slam_axes_locked()
    if axes[0] == "auto":
        return solve_best_transform(pairs, allow_scale=allow_scale)
    return solve_transform(pairs, allow_scale=allow_scale, slam_axes=axes)  # type: ignore[arg-type]


def _norm_deg(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def _slam_yaw_deg(yaw_val: float) -> float:
    """
    Heuristic: many senders report yaw in radians (typically within [-pi,pi]).
    If the magnitude looks radian-ish, convert to degrees.
    """
    import math

    y = float(yaw_val)
    if abs(y) <= (2.0 * math.pi + 0.5):
        y = math.degrees(y)
    return (y % 360.0 + 360.0) % 360.0


def _airsim_yaw_deg(pose: dict) -> float | None:
    try:
        if "yaw_deg" in pose:
            return float(pose["yaw_deg"])
        if "yaw" in pose:
            return float(pose["yaw"])
    except Exception:
        return None
    return None


def _read_slam_udp_pose(freshness_s: float) -> tuple[float, float, float, float, float] | None:
    """
    Returns (sx,sy,sz,yaw,ts) from SLAM UDP only.
    Alignment should never use keyframes (they can shift due to optimization/loop closure).
    """
    udp = get_udp_stats()
    try:
        packets = int(udp.get("packets", 0) or 0)
    except Exception:
        packets = 0
    age = udp.get("last_packet_age_s")
    if packets <= 0 or age is None or float(age) > freshness_s:
        return None
    try:
        sx = float(latest_pose.get("x", 0.0))
        sy = float(latest_pose.get("y", 0.0))
        sz = float(latest_pose.get("z", 0.0))
        yaw = float(latest_pose.get("yaw", 0.0))
        ts = float(latest_pose.get("ts", 0.0) or 0.0)
        ts_src = str(latest_pose.get("ts_source", "arrival"))
        if ts <= 0.0:
            return None
        # If the sender didn't provide a timestamp, ts is just local arrival time and
        # cannot be strictly aligned to AirSim pose timestamps.
        if ts_src != "sender":
            return None
        return (sx, sy, sz, yaw, ts)
    except Exception:
        return None


def _read_slam_pose_any_ts(freshness_s: float) -> dict | None:
    """
    Returns latest SLAM UDP pose even if ts_source is "arrival".
    Useful for non-calibration diagnostics/tests where strict sender timestamps aren't required.
    """
    udp = get_udp_stats()
    try:
        packets = int(udp.get("packets", 0) or 0)
    except Exception:
        packets = 0
    age = udp.get("last_packet_age_s")
    if packets <= 0 or age is None or float(age) > float(freshness_s):
        return None
    try:
        return {
            "x": float(latest_pose.get("x", 0.0)),
            "y": float(latest_pose.get("y", 0.0)),
            "z": float(latest_pose.get("z", 0.0)),
            "yaw": float(latest_pose.get("yaw", 0.0)),
            "ts": float(latest_pose.get("ts", 0.0) or 0.0),
            "ts_source": str(latest_pose.get("ts_source", "arrival")),
        }
    except Exception:
        return None

def _camera_offset_body_xyz() -> tuple[float, float, float]:
    """
    Body-frame offset from drone body reference to the stereo camera midpoint.
    AirSim NED body axes: +X forward, +Y right, +Z down.
    Defaults match the user's AirSim camera config (left/right at +/-0.1m Y).
    Only (x,y) are used for 2D pairing.
    """
    import os

    def f(name: str, default: float) -> float:
        try:
            return float(os.getenv(name, str(default)))
        except Exception:
            return default

    # Midpoint of the two stereo cameras.
    return (f("SLAM_CAMERA_OFF_X_M", 0.25), f("SLAM_CAMERA_OFF_Y_M", 0.0), f("SLAM_CAMERA_OFF_Z_M", -0.30))


def _airsim_body_to_camera_xy(ax: float, ay: float, yaw_deg: float) -> tuple[float, float]:
    """
    Convert AirSim body (ax,ay) to camera-center (ax_cam, ay_cam) in world frame.
    Uses yaw only (2D ground plane).
    """
    ox_b, oy_b, _oz_b = _camera_offset_body_xyz()
    a = math.radians(float(yaw_deg))
    ox = ox_b * math.cos(a) - oy_b * math.sin(a)
    oy = ox_b * math.sin(a) + oy_b * math.cos(a)
    return (float(ax) + ox, float(ay) + oy)


def _read_airsim_pose_for_align() -> tuple[float, float, float, float] | None:
    """
    Returns (ax, ay, yaw_deg, ts) from the published AirSim pose file.
    """
    pose = _read_airsim_pose_raw()
    try:
        ts = float(pose.get("ts", 0.0) or 0.0)
        if ts <= 0.0:
            return None
        ax = float(pose.get("x", 0.0))
        ay = float(pose.get("y", 0.0))
        yaw_deg = float(pose.get("yaw_deg", pose.get("yaw", 0.0)))
        return (ax, ay, yaw_deg, ts)
    except Exception:
        return None


# ---------------- Calibration (one-shot) ----------------
_calib_thread: threading.Thread | None = None
_calib_stop = threading.Event()
_calib_lock = threading.Lock()
_calib_state: dict = {
    "running": False,
    "pairs": 0,
    "rmse_m": None,
    "error": None,
    "status": None,
    "captured": 0,
    "skipped_missing": 0,
    "skipped_time": 0,
    "skipped_step": 0,
    "pattern": None,
    "config": None,
    "result": None,
}


def _set_calib_state(**kwargs) -> None:
    with _calib_lock:
        _calib_state.update(kwargs)


def _inc_calib_counter(name: str, inc: int = 1) -> None:
    with _calib_lock:
        _calib_state[name] = int(_calib_state.get(name, 0) or 0) + int(inc)


def _wait_until_close(target_x: float, target_y: float, tol_m: float, timeout_s: float) -> bool:
    t0 = time.time()
    while time.time() - t0 < timeout_s and not _calib_stop.is_set():
        pose = _read_airsim_pose_for_align()
        if pose is None:
            time.sleep(0.05)
            continue
        ax, ay, _yaw_deg, _ts = pose
        d = ((ax - target_x) ** 2 + (ay - target_y) ** 2) ** 0.5
        if d <= tol_m:
            return True
        time.sleep(0.05)
    return False


def _slam_axes_for_calibration_locked() -> tuple[Literal["x", "y", "z"], Literal["x", "y", "z"]]:
    # Per requirement: calibration solve uses locked axes (x,z).
    return ("x", "z")


def _slam_axes_for_calibration() -> tuple[Literal["x", "y", "z"], Literal["x", "y", "z"]] | Literal["auto"]:
    """
    Default locked axes (x,z), but allow explicit override for debugging:
      SLAM_CALIB_AXES=auto  -> choose best axis pairing
      SLAM_CALIB_AXES=x,z   -> locked (default)
    """
    raw = str(os.getenv("SLAM_CALIB_AXES", "x,z")).strip().lower().replace(" ", "")
    if raw in ("auto", "best"):
        return "auto"
    parts = [p for p in raw.split(",") if p]
    if len(parts) >= 2 and parts[0] in ("x", "y", "z") and parts[1] in ("x", "y", "z") and parts[0] != parts[1]:
        return (parts[0], parts[1])  # type: ignore[return-value]
    return _slam_axes_for_calibration_locked()


def _calib_max_time_s() -> float:
    # Stop-and-snap wants strict time pairing by default.
    try:
        return max(0.01, float(os.getenv("SLAM_CALIB_MAX_TIME_S", "0.10")))
    except Exception:
        return 0.10


def _calib_hover_s() -> float:
    try:
        return max(0.0, min(3.0, float(os.getenv("SLAM_CALIB_HOVER_S", "0.6"))))
    except Exception:
        return 0.6


def _calib_side_m() -> float:
    try:
        return max(2.0, float(os.getenv("SLAM_CALIB_SIDE_M", "8.0")))
    except Exception:
        return 8.0


def _calib_z_ned() -> float:
    try:
        return float(os.getenv("SLAM_CALIB_Z_NED", "-10.0"))
    except Exception:
        return -10.0


def _calibrate_snap_pair(*, require_sender_ts: bool) -> dict:
    """
    Shared implementation for stop-and-snap calibration.
    Returns the captured pair payload (also appended to pairs file).
    """
    # AirSim pose
    air = _read_airsim_pose_for_align()
    if air is None:
        raise HTTPException(status_code=503, detail="AirSim pose unavailable (airsim_pose.json missing/stale).")
    ax, ay, yaw_deg, air_ts = air

    # SLAM pose (UDP)
    slam_pose = _read_slam_udp_pose(freshness_s=_auto_align_params()["freshness_s"])
    if slam_pose is None:
        raise HTTPException(status_code=503, detail="No fresh SLAM UDP pose available.")
    sx, sy, sz, _syaw, slam_ts = slam_pose

    if require_sender_ts and str(latest_pose.get("ts_source", "arrival")) != "sender":
        raise HTTPException(status_code=409, detail="SLAM UDP must include sender timestamp (ts_source='sender').")

    max_dt_s = _calib_max_time_s()
    if abs(float(slam_ts) - float(air_ts)) > max_dt_s:
        raise HTTPException(status_code=409, detail=f"Time mismatch: |slam_ts-air_ts|={abs(float(slam_ts)-float(air_ts)):.3f}s > {max_dt_s:.3f}s")

    ax_cam, ay_cam = _airsim_body_to_camera_xy(ax, ay, yaw_deg)

    # Reject SLAM jumps (relocalization/map reset) that would poison calibration.
    # If SLAM moved a lot but AirSim camera barely moved, it's almost certainly a SLAM discontinuity.
    try:
        max_slam_jump_m = float(os.getenv("SLAM_CALIB_SNAP_MAX_SLAM_JUMP_M", "3.0"))
    except Exception:
        max_slam_jump_m = 3.0
    try:
        max_airsim_move_m = float(os.getenv("SLAM_CALIB_SNAP_MAX_AIRSIM_MOVE_M", "0.8"))
    except Exception:
        max_airsim_move_m = 0.8
    prev = load_pairs()
    if prev:
        p_last = prev[-1]
        ds = ((float(sx) - float(p_last.sx)) ** 2 + (float(sy) - float(p_last.sy)) ** 2 + (float(sz) - float(p_last.sz)) ** 2) ** 0.5
        da = ((float(ax_cam) - float(p_last.ax)) ** 2 + (float(ay_cam) - float(p_last.ay)) ** 2) ** 0.5
        if ds > max_slam_jump_m and da < max_airsim_move_m:
            raise HTTPException(status_code=409, detail=f"Rejected snap: SLAM jump detected (ds={ds:.2f}m, da={da:.2f}m).")

    pairs = append_pair(
        Pair(
            sx=float(sx),
            sy=float(sy),
            sz=float(sz),
            ax=float(ax_cam),
            ay=float(ay_cam),
            ax_body=float(ax),
            ay_body=float(ay),
            yaw_deg=float(yaw_deg),
        )
    )
    payload = {
        "sx": float(sx),
        "sy": float(sy),
        "sz": float(sz),
        "ax_cam": float(ax_cam),
        "ay_cam": float(ay_cam),
        "ax_body": float(ax),
        "ay_body": float(ay),
        "yaw_deg": float(yaw_deg),
        "airsim_ts": float(air_ts),
        "slam_ts": float(slam_ts),
    }
    return {"pairs": len(pairs), "pair": payload}


@router.post("/calibrate/snap")
def calibrate_snap():
    """
    Stop-and-snap calibration point:
    - requires SLAM sender timestamp
    - requires abs(slam_ts - airsim_ts) <= SLAM_CALIB_MAX_TIME_S (default 0.10s)
    - uses camera offset to pair AirSim camera-center pose
    """
    out = _calibrate_snap_pair(require_sender_ts=True)
    return {"ok": True, **out}


@router.post("/calibrate/run")
def calibrate_run():
    """
    Robust stop-and-snap calibration:
    - Clears pairs + transform
    - Flies a fixed waypoint pattern around start (rectangle + midpoints + center)
    - At each waypoint: goto -> wait -> hover -> sleep -> snap
    - Solves rigid transform (x,z) and saves it with RMSE
    """
    if get_slam_state() != "localization":
        return _calibration_requires_localization()

    set_slam_state("calibrating")
    reset_pairs()
    reset_transform()
    _calib_stop.clear()
    _set_calib_state(running=True, pairs=0, rmse_m=None, error=None, status="starting stop-and-snap", captured=0, skipped_missing=0, skipped_time=0, skipped_step=0)

    # Ensure systems are running and in correct modes.
    start_slam()
    start_drone()
    send_slam_mode("localization")
    send_drone_mode("explore")

    base = _read_airsim_pose_for_align()
    if base is None:
        _set_calib_state(running=False, status="error", error="AirSim pose unavailable for calibration.")
        raise HTTPException(status_code=503, detail="AirSim pose unavailable for calibration (airsim_pose.json missing/stale).")
    x0, y0, _yaw0, _t0 = base

    side = _calib_side_m()
    z_ned = _calib_z_ned()
    tol_m = 0.8
    timeout_s = 40.0
    hover_s = _calib_hover_s()
    try:
        goto_v = max(0.2, min(2.0, float(os.getenv("SLAM_CALIB_GOTO_VELOCITY_MPS", "0.6"))))
    except Exception:
        goto_v = 0.6

    pattern = str(os.getenv("SLAM_CALIB_RUN_PATTERN", "static_arc")).strip().lower().replace("-", "_")
    try:
        laps = max(1, int(os.getenv("SLAM_CALIB_LAPS", "2")))
    except Exception:
        laps = 2

    def _square_loop() -> list[tuple[float, float]]:
        # Mimics the static mapping "lap" shape (4 legs around a square), but stop-and-snap at corners.
        one = [
            (x0, y0),
            (x0 + side, y0),
            (x0 + side, y0 + side),
            (x0, y0 + side),
            (x0, y0),
        ]
        out: list[tuple[float, float]] = []
        for _ in range(laps):
            out.extend(one)
        return out

    def _cross() -> list[tuple[float, float]]:
        # Out-and-back on X then on Y (still stop-and-snap).
        return [
            (x0, y0),
            (x0 + side, y0),
            (x0, y0),
            (x0, y0 + side),
            (x0, y0),
        ]

    def _rectangle_center() -> list[tuple[float, float]]:
        # Older pattern: rectangle + midpoints + center (>=8 stops).
        return [
            (x0, y0),
            (x0 + side, y0),
            (x0 + side, y0 + side),
            (x0, y0 + side),
            (x0, y0),
            (x0 + 0.5 * side, y0),
            (x0 + side, y0 + 0.5 * side),
            (x0 + 0.5 * side, y0 + side),
            (x0, y0 + 0.5 * side),
            (x0 + 0.5 * side, y0 + 0.5 * side),
        ]

    if pattern in ("static_arc", "static_turns", "arc"):
        pts = []
    elif pattern in ("static", "static_square", "square"):
        pts = _square_loop()
    elif pattern in ("cross", "out_and_back"):
        pts = _cross()
    else:
        pts = _rectangle_center()

    _set_calib_state(
        status="flying waypoints + snapping",
        config={
            "mode": "stop_and_snap",
            "pattern": pattern,
            "laps": int(laps),
            "side_m": float(side),
            "z_ned": float(z_ned),
            "tol_m": float(tol_m),
            "timeout_s": float(timeout_s),
            "hover_s": float(hover_s),
            "goto_velocity_mps": float(goto_v),
            "max_time_s": float(_calib_max_time_s()),
            "waypoints": [{"x": float(x), "y": float(y)} for x, y in pts],
        },
    )

    try:
        if pattern in ("static_arc", "static_turns", "arc"):
            # Mimic static mapping motion (forward legs + arc turns), but smaller, and stop-and-snap at corners.
            _set_calib_state(status="flying static-arc lap(s) + snapping")
            send_calib_static_run(side_m=float(side), z=float(z_ned), speed_mps=float(goto_v), laps=int(laps), hover_s=float(hover_s))

            last_corner = None
            t_start = time.time()
            overall_timeout_s = max(60.0, float(laps) * 4.0 * float(timeout_s))
            while not _calib_stop.is_set() and (time.time() - t_start) < overall_timeout_s:
                st = _read_calib_motion_status()
                phase = str(st.get("phase")) if st.get("phase") is not None else None
                if phase == "hover":
                    corner = st.get("corner")
                    if corner is not None and corner != last_corner:
                        # Retry snapping a few times to ride out brief timing jitter / SLAM relocalization.
                        snap = None
                        last_err = None
                        for _attempt in range(12):
                            try:
                                snap = _calibrate_snap_pair(require_sender_ts=True)
                                break
                            except HTTPException as he:
                                if int(getattr(he, "status_code", 0) or 0) in (409, 503):
                                    last_err = str(getattr(he, "detail", "")) or str(he)
                                    time.sleep(0.12)
                                    continue
                                raise
                        if snap is None:
                            raise RuntimeError(f"Failed to snap at corner {corner}: {last_err or 'unknown'}")
                        last_corner = corner
                        _set_calib_state(pairs=int(snap["pairs"]), captured=int(snap["pairs"]))
                if phase in ("done", "stop"):
                    break
                time.sleep(0.05)
        else:
            for i, (x, y) in enumerate(pts):
                if _calib_stop.is_set():
                    raise RuntimeError("Calibration aborted")
                send_goto(float(x), float(y), float(z_ned), velocity=float(goto_v))
                ok = _wait_until_close(float(x), float(y), tol_m=tol_m, timeout_s=timeout_s)
                if not ok:
                    raise RuntimeError(f"Timeout reaching waypoint {i+1}/{len(pts)}")
                send_hover()
                if hover_s:
                    time.sleep(hover_s)
                # Retry snapping a few times to ride out brief timing jitter / SLAM relocalization.
                snap = None
                last_err = None
                for _attempt in range(12):
                    try:
                        snap = _calibrate_snap_pair(require_sender_ts=True)
                        break
                    except HTTPException as he:
                        # Only retry for "soft" failures.
                        if int(getattr(he, "status_code", 0) or 0) in (409, 503):
                            last_err = str(getattr(he, "detail", "")) or str(he)
                            time.sleep(0.12)
                            continue
                        raise
                if snap is None:
                    raise RuntimeError(f"Failed to snap at waypoint {i+1}/{len(pts)}: {last_err or 'unknown'}")
                _set_calib_state(pairs=int(snap["pairs"]), captured=int(snap["pairs"]))

        pairs = load_pairs()
        if len(pairs) < 8:
            raise RuntimeError(f"Not enough pairs captured ({len(pairs)}), need >= 8.")
        span = _span_m(pairs)
        if span < 10.0:
            raise RuntimeError(f"Not enough spatial span for a stable solve (span={span:.1f}m, need ≥10.0m). Increase SLAM_CALIB_SIDE_M.")

        _set_calib_state(status="solving transform")
        t, e, used = solve_transform_robust(pairs, allow_scale=True)
        save_transform(t, pairs_used=int(used), rmse_m=float(e), quality=_rmse_quality(float(e)) or "unknown", ts=time.time())
        _set_calib_state(
            status="done",
            rmse_m=float(e),
            pairs=len(pairs),
            error=None,
            result={
                "rmse_m": float(e),
                "quality": _rmse_quality(float(e)),
                "airsim_span_m": float(span),
                "transform": _transform_summary(t),
            },
        )
        return {
            "ok": True,
            "rmse_m": float(e),
            "quality": _rmse_quality(float(e)),
            "scale": float(t.scale),
            "slam_axes": list(t.slam_axes),
            "transform": _transform_summary(t),
            "pairs_used": int(used),
            "pairs_total": len(pairs),
        }
    except HTTPException:
        raise
    except Exception as e:
        _set_calib_state(status="error", error=f"{type(e).__name__}: {e}")
        raise HTTPException(status_code=400, detail=str(_calib_state.get("error")))
    finally:
        _set_calib_state(running=False)
        set_slam_state("localization")


@router.post("/calibrate/solve")
def calibrate_solve():
    """
    Solve using currently captured stop-and-snap pairs:
    - rigid transform only (allow_scale=False)
    - locked axes (x,z)
    - saves transform with RMSE
    """
    pairs = load_pairs()
    if len(pairs) < 2:
        raise HTTPException(status_code=400, detail="Need at least 2 pairs to solve.")
    span = _span_m(pairs)
    if len(pairs) < 8 or span < 10.0:
        raise HTTPException(status_code=400, detail=f"Need >=8 pairs and >=10m span (pairs={len(pairs)}, span={span:.1f}m).")
    t, e, used = solve_transform_robust(pairs, allow_scale=True)
    save_transform(t, pairs_used=int(used), rmse_m=float(e), quality=_rmse_quality(float(e)) or "unknown", ts=time.time())
    return {
        "ok": True,
        "pairs_used": int(used),
        "pairs_total": len(pairs),
        "rmse_m": float(e),
        "quality": _rmse_quality(float(e)),
        "scale": float(t.scale),
        "slam_axes": list(t.slam_axes),
        "transform": _transform_summary(t),
    }


@router.post("/calibrate/solve_recorded")
def calibrate_solve_recorded():
    """
    Solve SLAM->AirSim transform from already recorded pairs (e.g., collected during a mission),
    without starting any new motion.
    """
    if get_slam_state() != "localization":
        return _calibration_requires_localization()

    pairs = load_pairs()
    if len(pairs) < 2:
        msg = "Need at least 2 recorded pairs to solve."
        _set_calib_state(running=False, status="error", pairs=len(pairs), error=msg)
        raise HTTPException(status_code=400, detail=msg)

    # Use the same minimums as stop-and-snap by default (can be relaxed via env if needed).
    span = _span_m(pairs)
    min_pairs = int(os.getenv("SLAM_RECORD_SOLVE_MIN_PAIRS", "12"))
    min_span_m = float(os.getenv("SLAM_RECORD_SOLVE_MIN_SPAN_M", "10.0"))
    if len(pairs) < max(2, min_pairs) or span < min_span_m:
        msg = f"Not enough recorded data to solve (pairs={len(pairs)}, span={span:.1f}m). Need >= {min_pairs} pairs and >= {min_span_m:.1f}m span."
        _set_calib_state(running=False, status="error", pairs=len(pairs), error=msg)
        raise HTTPException(status_code=400, detail=msg)

    t, e, used = solve_transform_robust(pairs, allow_scale=True)
    save_transform(t, pairs_used=int(used), rmse_m=float(e), quality=_rmse_quality(float(e)) or "unknown", ts=time.time())
    _set_calib_state(
        running=False,
        status="done",
        rmse_m=float(e),
        pairs=len(pairs),
        error=None,
        result={
            "rmse_m": float(e),
            "quality": _rmse_quality(float(e)),
            "airsim_span_m": float(span),
            "transform": _transform_summary(t),
        },
    )
    return {
        "ok": True,
        "slam_state": get_slam_state(),
        "pairs_used": int(used),
        "pairs_total": len(pairs),
        "rmse_m": float(e),
        "quality": _rmse_quality(float(e)),
        "transform": _transform_summary(t),
        "scale": float(t.scale),
        "slam_axes": list(t.slam_axes),
        "msg": "Calibration solved from recorded mission pairs.",
    }


class ForwardTestReq(BaseModel):
    dist_m: float = 10.0
    speed_mps: float = 1.0
    z_ned: float | None = None
    freshness_s: float = 2.0


@router.post("/test/forward")
def test_forward(req: ForwardTestReq):
    """
    Command the drone_motion process to move forward by dist_m (body frame),
    then report the measured displacement magnitude in SLAM coordinates.
    """
    if get_slam_state() == "calibrating":
        return JSONResponse(status_code=409, content={"ok": False, "error": "Calibration is running. Wait for it to finish."})

    st = slam_status()
    if not bool(st.get("running")):
        raise HTTPException(status_code=400, detail="SLAM must be running for this test.")

    dist_m = max(0.5, min(50.0, float(req.dist_m)))
    speed_mps = max(0.2, min(3.0, float(req.speed_mps)))
    freshness_s = max(0.2, min(10.0, float(req.freshness_s)))

    slam0 = _read_slam_pose_any_ts(freshness_s=freshness_s)
    if slam0 is None:
        raise HTTPException(status_code=503, detail="No fresh SLAM UDP pose available.")
    air0 = _read_airsim_pose_raw()
    ax0 = float(air0.get("x", 0.0))
    ay0 = float(air0.get("y", 0.0))
    az0 = float(air0.get("z", -10.0))

    z_ned = float(req.z_ned) if req.z_ned is not None else float(az0)

    # Put drone motion into explore and execute forward motion.
    start_drone()
    send_drone_mode("explore")
    send_forward(dist_m=dist_m, speed_mps=speed_mps, z=z_ned)

    # Wait until AirSim has moved ~dist_m (or timeout).
    t0 = time.time()
    timeout_s = max(8.0, (dist_m / speed_mps) * 2.5 + 5.0)
    reached = False
    while (time.time() - t0) < timeout_s:
        air = _read_airsim_pose_raw()
        ax = float(air.get("x", 0.0))
        ay = float(air.get("y", 0.0))
        d = ((ax - ax0) ** 2 + (ay - ay0) ** 2) ** 0.5
        if d >= (0.9 * dist_m):
            reached = True
            break
        time.sleep(0.2)

    slam1 = _read_slam_pose_any_ts(freshness_s=freshness_s)
    if slam1 is None:
        raise HTTPException(status_code=503, detail="SLAM UDP pose went stale during test.")

    air1 = _read_airsim_pose_raw()
    ax1 = float(air1.get("x", 0.0))
    ay1 = float(air1.get("y", 0.0))
    az1 = float(air1.get("z", az0))

    dxs = float(slam1["x"]) - float(slam0["x"])
    dys = float(slam1["y"]) - float(slam0["y"])
    dzs = float(slam1["z"]) - float(slam0["z"])
    slam_dist = float((dxs * dxs + dys * dys + dzs * dzs) ** 0.5)

    dxa = ax1 - ax0
    dya = ay1 - ay0
    dza = az1 - az0
    air_dist = float((dxa * dxa + dya * dya + dza * dza) ** 0.5)

    ratio = None
    if air_dist > 1e-6:
        ratio = slam_dist / air_dist

    return {
        "ok": True,
        "reached": reached,
        "requested_dist_m": dist_m,
        "airsim_dist_m": air_dist,
        "slam_dist_m": slam_dist,
        "slam_over_airsim_scale": ratio,
        "slam_delta": {"dx": dxs, "dy": dys, "dz": dzs},
        "airsim_delta": {"dx": dxa, "dy": dya, "dz": dza},
        "slam_start": slam0,
        "slam_end": slam1,
        "airsim_start": {"x": ax0, "y": ay0, "z": az0},
        "airsim_end": {"x": ax1, "y": ay1, "z": az1},
    }


@router.post("/calibrate/solve_best")
def calibrate_solve_best(allow_scale: bool = False):
    """
    Solve using currently captured stop-and-snap pairs, but choose the best SLAM ground-plane axes.
    This is the fastest way to confirm an axis mismatch (e.g. (x,y) vs (x,z)).
    """
    pairs = load_pairs()
    if len(pairs) < 2:
        raise HTTPException(status_code=400, detail="Need at least 2 pairs to solve.")
    span = _span_m(pairs)
    if len(pairs) < 8 or span < 10.0:
        raise HTTPException(status_code=400, detail=f"Need >=8 pairs and >=10m span (pairs={len(pairs)}, span={span:.1f}m).")
    t = solve_best_transform(pairs, allow_scale=bool(allow_scale))
    e = float(rmse_m(t, pairs))
    save_transform(t, pairs_used=len(pairs), rmse_m=float(e), quality=_rmse_quality(float(e)) or "unknown", ts=time.time())
    return {"ok": True, "pairs_used": len(pairs), "rmse_m": float(e), "quality": _rmse_quality(float(e)), "transform": _transform_summary(t)}


def _capture_pair_if_ready(last_sample_xy: tuple[float, float] | None, sample_step_m: float, max_dt_s: float) -> tuple[bool, tuple[float, float] | None]:
    """
    If moved enough since last_sample_xy and timestamps match, append a camera-corrected pair.
    Returns (captured, new_last_sample_xy).
    """
    air = _read_airsim_pose_for_align()
    slam = _read_slam_udp_pose(freshness_s=_auto_align_params()["freshness_s"])
    if air is None or slam is None:
        _inc_calib_counter("skipped_missing")
        return (False, last_sample_xy)
    ax, ay, ayaw_deg, at = air
    sx, sy, sz, syaw, st = slam
    if abs(float(st) - float(at)) > float(max_dt_s):
        _inc_calib_counter("skipped_time")
        return (False, last_sample_xy)
    if last_sample_xy is not None:
        dx = ax - last_sample_xy[0]
        dy = ay - last_sample_xy[1]
        if (dx * dx + dy * dy) ** 0.5 < sample_step_m:
            _inc_calib_counter("skipped_step")
            return (False, last_sample_xy)
    ax_cam, ay_cam = _airsim_body_to_camera_xy(ax, ay, ayaw_deg)
    append_pair(
        Pair(
            sx=float(sx),
            sy=float(sy),
            sz=float(sz),
            ax=float(ax_cam),
            ay=float(ay_cam),
            ax_body=float(ax),
            ay_body=float(ay),
            yaw_deg=float(ayaw_deg),
        )
    )
    pairs = load_pairs()
    _set_calib_state(
        pairs=len(pairs),
        pairs_collected=len(pairs),
        pairs_count=len(pairs),
    )
    _inc_calib_counter("captured")
    return (True, (ax, ay))


def _read_calib_motion_phase() -> str | None:
    try:
        if _CALIB_MOTION_STATUS_FILE.exists():
            d = json.loads(_CALIB_MOTION_STATUS_FILE.read_text())
            if isinstance(d, dict):
                return str(d.get("phase")) if d.get("phase") is not None else None
    except Exception:
        return None
    return None


def _read_calib_motion_status() -> dict:
    try:
        if _CALIB_MOTION_STATUS_FILE.exists():
            d = json.loads(_CALIB_MOTION_STATUS_FILE.read_text())
            return d if isinstance(d, dict) else {}
    except Exception:
        pass
    return {}


@router.post("/calibrate/start")
def calibrate_start():
    """
    One-shot calibration:
    - Clears alignment pairs + transform
    - Flies a fixed square path (8m per side by default)
    - Captures pose pairs every ~2m using:
        SLAM UDP pose (must include sender timestamp) <-> AirSim camera-center pose (body pose + camera offset)
    - Solves a single rigid transform with locked axes (default x,z)
    """
    global _calib_thread
    if _calib_thread and _calib_thread.is_alive():
        return {"ok": True, "running": True, "slam_state": get_slam_state(), **_calib_state}

    if get_slam_state() != "localization":
        return _calibration_requires_localization()

    # Stop auto-align if running (calibration is one-shot, not continuous).
    try:
        _auto_align_stop.set()
    except Exception:
        pass

    reset_pairs()
    reset_transform()
    _calib_stop.clear()
    set_slam_state("calibrating")
    _set_calib_state(
        running=True,
        pairs=0,
        rmse_m=None,
        error=None,
        status="starting",
        captured=0,
        skipped_missing=0,
        skipped_time=0,
        skipped_step=0,
        pattern=None,
        config=None,
        result=None,
    )

    import os

    # Default to a 2D path (square+diagonals) for observability.
    side_m = float(os.getenv("SLAM_CALIB_SIDE_M", "12.0"))
    sample_step_m = float(os.getenv("SLAM_CALIB_SAMPLE_M", "2.0"))
    max_dt_s = float(os.getenv("SLAM_CALIB_MAX_TIME_S", "0.25"))
    tol_m = float(os.getenv("SLAM_CALIB_TOL_M", "1.0"))
    leg_timeout_s = float(os.getenv("SLAM_CALIB_LEG_TIMEOUT_S", "40.0"))
    min_pairs = int(os.getenv("SLAM_CALIB_MIN_PAIRS", "10"))
    min_span_m = float(os.getenv("SLAM_CALIB_MIN_SPAN_M", "10.0"))
    # Calibration-only motion parameters (executed by drone_motion, isolated from other modes).
    # Defaults match an "out-and-back" pattern:
    #   rotate to 0° -> forward SIDE -> rotate to 180° -> forward SIDE (return).
    # NED: negative is UP. Default to a safer higher altitude for calibration.
    z_ned = float(os.getenv("SLAM_CALIB_Z_NED", "-10.0"))
    speed_mps = float(os.getenv("SLAM_CALIB_VELOCITY_MPS", "0.7"))
    pause_s = float(os.getenv("SLAM_CALIB_PAUSE_S", "0.7"))
    repeats = int(os.getenv("SLAM_CALIB_REPEATS", "1"))
    pattern = str(os.getenv("SLAM_CALIB_PATTERN", "square_diagonals")).strip().lower().replace("-", "_")

    def _calib_path(pattern_name: str, side: float) -> tuple[list[float], list[float]]:
        s = max(1.0, float(side))
        diag = s * (2.0**0.5)
        if pattern_name in ("out_and_back", "outback", "line"):
            return ([0.0, 180.0], [s, s])
        if pattern_name in ("square", "box"):
            return ([0.0, 90.0, 180.0, 270.0], [s, s, s, s])
        # default: square + both diagonals (returns to start)
        yaws = [0.0, 90.0, 180.0, 270.0, 45.0, 225.0, 135.0, 315.0]
        dists = [s, s, s, s, diag, diag, diag, diag]
        return (yaws, dists)

    yaw_seq_deg, dist_seq_m = _calib_path(pattern, side_m)
    _set_calib_state(
        pattern=pattern,
        config={
            "side_m": float(side_m),
            "sample_step_m": float(sample_step_m),
            "max_dt_s": float(max_dt_s),
            "min_pairs": int(min_pairs),
            "min_span_m": float(min_span_m),
            "z_ned": float(z_ned),
            "speed_mps": float(speed_mps),
            "pause_s": float(pause_s),
            "repeats": int(repeats),
            "yaw_seq_deg": [float(y) for y in yaw_seq_deg],
            "dist_seq_m": [float(d) for d in dist_seq_m],
        },
    )

    # Preflight (fast fail): if we have no SLAM UDP packets, calibration cannot work.
    # This avoids the "nothing is happening" experience.
    slam_start = start_slam()
    if isinstance(slam_start, dict) and slam_start.get("ok") is False and "already running" not in str(slam_start.get("msg", "")).lower():
        _set_calib_state(running=False, status="error", error=str(slam_start.get("msg") or "SLAM failed to start"))
        return {"ok": False, "error": _calib_state.get("error"), "slam": slam_start}

    try:
        start_drone()
    except Exception:
        pass

    # Wait briefly for SLAM UDP to start streaming with sender timestamps.
    t0 = time.time()
    while time.time() - t0 < 3.0:
        udp = get_udp_stats()
        try:
            if int(udp.get("packets", 0) or 0) > 0 and str(latest_pose.get("ts_source", "arrival")) == "sender":
                break
        except Exception:
            pass
        time.sleep(0.1)
    else:
        udp = get_udp_stats()
        _set_calib_state(running=False, status="error", error="No SLAM UDP packets with sender timestamp; calibration cannot proceed.")
        return {
            "ok": False,
            "error": _calib_state.get("error"),
            "udp": udp,
            "hint": "Start ORB-SLAM bridge and ensure it sends 'sx sy sz yaw slam_ts' (epoch seconds) to 127.0.0.1:5005.",
        }

    def worker():
        try:
            # Require sender timestamps for calibration.
            if str(latest_pose.get("ts_source", "arrival")) != "sender":
                raise RuntimeError("SLAM UDP must include sender timestamp (sx sy sz yaw slam_ts).")

            # Ensure systems are running.
            start_slam()
            start_drone()
            send_slam_mode("localization")
            send_drone_mode("explore")

            base = _read_airsim_pose_for_align()
            if base is None:
                raise RuntimeError("AirSim pose unavailable for calibration (airsim_pose.json missing/stale).")
            x0, y0, _yaw0, _t0 = base

            _set_calib_state(status=f"flying calibration path ({pattern})")
            send_calib_run(
                side_m=float(side_m),
                z=float(z_ned),
                speed_mps=float(speed_mps),
                repeats=repeats,
                pause_s=float(pause_s),
                yaw_seq_deg=yaw_seq_deg,
                dist_seq_m=dist_seq_m,
            )

            # Pair capture loop: poll while calibration motion runs.
            last_sample: tuple[float, float] | None = None
            t_start = time.time()
            overall_timeout_s = max(30.0, float(repeats) * 4.0 * (float(side_m) / max(0.2, float(speed_mps))) * 2.0)
            while not _calib_stop.is_set() and (time.time() - t_start) < overall_timeout_s:
                phase = _read_calib_motion_phase()
                # Do NOT capture during hover/rotate phases.
                if phase == "forward":
                    _captured, last_sample = _capture_pair_if_ready(last_sample, sample_step_m=sample_step_m, max_dt_s=max_dt_s)
                # Exit when motion reports done.
                if phase == "done" or phase == "stop":
                    break
                time.sleep(0.05)

            try:
                send_hover()
            except Exception:
                pass

            pairs = load_pairs()
            if len(pairs) < max(10, int(min_pairs)):
                raise RuntimeError(f"Not enough pairs captured ({len(pairs)}). Increase motion span or reduce time/yaw gating.")

            span = _span_m(pairs)
            if span < float(min_span_m):
                raise RuntimeError(f"Not enough spatial span for a stable solve (span={span:.1f}m, need ≥{float(min_span_m):.1f}m). Increase SLAM_CALIB_SIDE_M.")

            axes = _slam_axes_locked()
            _set_calib_state(status="solving transform")
            t = _solve_transform_from_pairs(pairs, allow_scale=False)
            e = rmse_m(t, pairs)
            save_transform(t, pairs_used=len(pairs), rmse_m=float(e), quality=_rmse_quality(float(e)) or "unknown", ts=time.time())
            _set_calib_state(
                status="done",
                rmse_m=float(e),
                pairs=len(pairs),
                error=None,
                result={
                    "rmse_m": float(e),
                    "quality": _rmse_quality(float(e)),
                    "airsim_span_m": float(span),
                    "slam_span_3d_m": float(_slam_span_3d(pairs)),
                    "transform": _transform_summary(t),
                },
            )
        except Exception as e:
            _set_calib_state(status="error", error=f"{type(e).__name__}: {e}")
        finally:
            _set_calib_state(running=False)
            set_slam_state("localization")

    _calib_thread = threading.Thread(target=worker, daemon=True)
    _calib_thread.start()
    return {"ok": True, "running": True, "msg": "calibration started", "pattern": pattern, "legs": len(yaw_seq_deg)}


@router.post("/calibrate/stop")
def calibrate_stop():
    _calib_stop.set()
    try:
        send_calib_stop()
    except Exception:
        pass
    return {"ok": True, "running": False}


@router.get("/calibrate/status")
def calibrate_status():
    with _calib_lock:
        phase = _read_calib_motion_phase()
        return {"ok": True, "slam_state": get_slam_state(), "motion_phase": phase, **_calib_state}


def _airsim_body_to_camera_xy_with_offset(ax: float, ay: float, yaw_deg: float, ox_b: float, oy_b: float) -> tuple[float, float]:
    a = math.radians(float(yaw_deg))
    ox = float(ox_b) * math.cos(a) - float(oy_b) * math.sin(a)
    oy = float(ox_b) * math.sin(a) + float(oy_b) * math.cos(a)
    return (float(ax) + ox, float(ay) + oy)


def _transform_summary(t) -> dict:
    # Rotation angle from 2D rotation matrix.
    yaw_deg = math.degrees(math.atan2(float(t.r21), float(t.r11)))
    return {
        "model": t.model,
        "slam_axes": list(t.slam_axes),
        "scale": float(t.scale),
        "yaw_deg": float(_norm_deg(float(yaw_deg))),
        "tx": float(t.tx),
        "ty": float(t.ty),
        "r": [[float(t.r11), float(t.r12)], [float(t.r21), float(t.r22)]],
    }


@router.get("/calibrate/diagnostics")
def calibrate_diagnostics():
    """
    Offline diagnostics from the currently captured pairs:
    - Solve with camera-offset enabled (using current SLAM_CAMERA_OFF_{X,Y}_M)
    - Solve with offset disabled (treat AirSim body pose as the paired point)

    This helps identify systematic bias caused by camera-offset rotation (yaw convention mismatch).
    """
    pairs = load_pairs()
    if len(pairs) < 2:
        raise HTTPException(status_code=400, detail="Not enough pairs. Run /slam/calibrate/start first.")

    raw_missing = any((p.ax_body is None or p.ay_body is None or p.yaw_deg is None) for p in pairs)
    if raw_missing:
        raise HTTPException(
            status_code=400,
            detail="Pairs file is missing raw AirSim pose metadata (ax_body/ay_body/yaw_deg). Re-run /slam/calibrate/start after updating the server.",
        )

    ox_b, oy_b, _oz_b = _camera_offset_body_xyz()
    axes = _slam_axes_locked()

    def _recompute(ox: float, oy: float) -> list[Pair]:
        out: list[Pair] = []
        for p in pairs:
            ax_body = float(p.ax_body)  # type: ignore[arg-type]
            ay_body = float(p.ay_body)  # type: ignore[arg-type]
            yaw_deg = float(p.yaw_deg)  # type: ignore[arg-type]
            ax, ay = _airsim_body_to_camera_xy_with_offset(ax_body, ay_body, yaw_deg, ox, oy)
            out.append(Pair(sx=p.sx, sy=p.sy, sz=p.sz, ax=ax, ay=ay, ax_body=ax_body, ay_body=ay_body, yaw_deg=yaw_deg))
        return out

    pairs_on = _recompute(float(ox_b), float(oy_b))
    pairs_off = _recompute(0.0, 0.0)

    t_on = _solve_transform_from_pairs(pairs_on, allow_scale=False)
    e_on = float(rmse_m(t_on, pairs_on))
    t_off = _solve_transform_from_pairs(pairs_off, allow_scale=False)
    e_off = float(rmse_m(t_off, pairs_off))

    better = "offset_on" if e_on <= e_off else "offset_off"
    suggestion = None
    if better == "offset_off" and (e_on - e_off) >= 0.5:
        suggestion = "RMSE improves with offset disabled; likely camera-offset rotation (yaw convention) or offset values are wrong."
    elif better == "offset_on" and (e_off - e_on) >= 0.5:
        suggestion = "RMSE improves with offset enabled; keep offset and tune SLAM_CAMERA_OFF_{X,Y}_M if needed."

    return {
        "ok": True,
        "pairs": len(pairs),
        "slam_axes_requested": list(axes),
        "slam_axes_used_offset_on": list(t_on.slam_axes),
        "slam_axes_used_offset_off": list(t_off.slam_axes),
        "offset_used_xy_m": {"x": float(ox_b), "y": float(oy_b)},
        "offset_on": {"rmse_m": e_on, "span_m": _span_m(pairs_on), "transform": _transform_summary(t_on)},
        "offset_off": {"rmse_m": e_off, "span_m": _span_m(pairs_off), "transform": _transform_summary(t_off)},
        "better": better,
        "suggestion": suggestion,
    }

# ---------- START/STOP ----------
@router.post("/start")
def start_all():
    # Starting SLAM creates a new SLAM world frame (origin/orientation can reset).
    # To avoid applying stale transforms, reset alignment pairs + transform on start.
    reset_pairs()
    reset_transform()
    start_slam()
    start_drone()
    send_slam_mode("mapping")
    # Ensure mapping resumes if the drone was previously stopped (explore mode).
    send_drone_mode("mapping")
    set_slam_state("mapping")
    return {"ok": True, "msg": "SLAM + Drone started (alignment reset)"}

@router.post("/stop")
def stop_all():
    # Stop the drone's motion without terminating the drone_motion process.
    send_hover()
    stop_slam()
    return {"ok": True, "msg": "SLAM + Drone stopped"}

# ---------- STATUS ----------
@router.get("/status")
def status():
    return slam_status()


# ---------- SLAM SESSION ----------
@router.get("/session/status")
def session_status():
    return {"ok": True, "slam_state": get_slam_state()}


@router.post("/session/finalize_map")
def session_finalize_map():
    st = get_slam_state()
    if st == "calibrating":
        return JSONResponse(status_code=409, content={"ok": False, "error": "Calibration is running. Wait for it to finish."})
    if st == "localization":
        return {"ok": True, "slam_state": "localization", "msg": "SLAM map finalized. Localization mode active."}
    # mapping -> localization (no restart/reset; just a mode switch)
    send_slam_mode("localization")
    set_slam_state("localization")
    # Stop any active dynamic mission (coverage mapping) so the operator can take over in localization.
    try:
        send_mission_abort(mission_id="")
    except Exception:
        pass
    try:
        stop_mission_pair_recording()
    except Exception:
        pass
    return {"ok": True, "slam_state": "localization", "msg": "SLAM map finalized. Localization mode active."}

@router.get("/pose")
def pose():
    return latest_pose

@router.get("/trajectory")
def trajectory():
    # Keyframes-based trajectory (ORB-SLAM3) is the most stable reference for
    # visualization when tracking is lost/relocalized.
    return read_keyframes()

@router.get("/udp")
def udp():
    return get_udp_stats()

@router.get("/keyframes")
def keyframes():
    return read_keyframes()

# ---------- GOTO ----------
class GotoReq(BaseModel):
    x: float
    y: float = 0.0
    z: float

@router.post("/goto")
def goto(req: GotoReq):
    send_goto(req.x, req.y, req.z)
    return {"ok": True, "msg": "goto sent"}

def _read_airsim_xy() -> tuple[float, float]:
    # Prefer pose published by drone_motion (thread-safe vs AirSim RPC).
    try:
        if _AIRSIM_POSE_FILE.exists():
            d = json.loads(_AIRSIM_POSE_FILE.read_text())
            return (float(d["x"]), float(d["y"]))
    except Exception:
        pass
    raise HTTPException(status_code=503, detail="AirSim pose unavailable (airsim_pose.json missing)")

def _read_airsim_pose_raw() -> dict:
    try:
        if _AIRSIM_POSE_FILE.exists():
            d = json.loads(_AIRSIM_POSE_FILE.read_text())
            return d if isinstance(d, dict) else {}
    except Exception:
        pass
    return {}

def _read_slam_pose_for_alignment(freshness_s: float) -> tuple[float, float, float, str] | None:
    """
    Backwards-compatible wrapper used by some older UI paths.
    For calibration/alignment we ONLY accept UDP poses (see _read_slam_udp_pose()).
    """
    p = _read_slam_udp_pose(freshness_s)
    if p is None:
        return None
    sx, sy, sz, _yaw, _ts = p
    return (sx, sy, sz, "udp")


# ---------------- Auto-alignment (optional) ----------------
_auto_align_thread: threading.Thread | None = None
_auto_align_stop = threading.Event()
_auto_align_lock = threading.Lock()
_auto_align_state: dict = {
    "running": False,
    "pairs_collected": 0,
    "last_solve_rmse_m": None,
    "last_solve_ts": None,
    "last_error": None,
    "last_attempt_rmse_m": None,
    "last_attempt_axes": None,
    "last_attempt_model": None,
    "slam_udp_packets": 0,
    "slam_last_packet_age_s": None,
    "skipped_time": 0,
    "skipped_yaw": 0,
    "skipped_step": 0,
}

_auto_align_history: list[dict] = []
_AUTO_ALIGN_HISTORY_MAX = 600  # ~10 minutes at 1Hz

_snap_wizard: dict = {
    "enabled": False,
    # Calibration-safe defaults (stop-and-snap).
    "freshness_s": 1.5,
    "snap_max_vxy_mps": 0.25,
    "snap_max_slam_jump_m": 3.0,
    "snap_max_airsim_move_m": 0.8,
}


@router.post("/align/wizard/start")
def align_wizard_start():
    # Stop auto-align if running; wizard uses manual snaps.
    try:
        _auto_align_stop.set()
    except Exception:
        pass
    _snap_wizard["enabled"] = True
    return {"ok": True, "enabled": True, "settings": dict(_snap_wizard)}


@router.post("/align/wizard/stop")
def align_wizard_stop():
    _snap_wizard["enabled"] = False
    return {"ok": True, "enabled": False}


@router.get("/align/wizard/status")
def align_wizard_status():
    # Report if basic telemetry is fresh enough for good snaps.
    udp = get_udp_stats()
    pose = _read_airsim_pose_raw()
    pose_age = None
    try:
        ts = float(pose.get("ts", 0.0) or 0.0)
        if ts > 0:
            pose_age = time.time() - ts
    except Exception:
        pose_age = None
    return {
        "ok": True,
        "enabled": bool(_snap_wizard.get("enabled")),
        "settings": dict(_snap_wizard),
        "udp": udp,
        "airsim_pose_age_s": pose_age,
    }


def _auto_align_params() -> dict:
    import os

    def f(name: str, default: float) -> float:
        try:
            return float(os.getenv(name, str(default)))
        except Exception:
            return default

    def i(name: str, default: int) -> int:
        try:
            return int(os.getenv(name, str(default)))
        except Exception:
            return default

    return {
        "interval_s": max(0.1, f("SLAM_ALIGN_INTERVAL_S", 0.5)),
        # Default tuned for small-area missions: learn quickly but still avoid jitter.
        "min_step_m": max(0.05, f("SLAM_ALIGN_MIN_STEP_M", 0.5)),
        # Reject likely relocalization jumps (large discontinuities) so they don't poison the fit.
        "max_step_m": max(0.5, f("SLAM_ALIGN_MAX_STEP_M", 6.0)),
        "max_pairs": max(20, i("SLAM_ALIGN_MAX_PAIRS", 120)),
        "min_pairs_to_solve": max(3, i("SLAM_ALIGN_MIN_PAIRS", 6)),
        "min_span_m": max(0.5, f("SLAM_ALIGN_MIN_SPAN_M", 3.0)),
        "max_rmse_m": max(0.1, f("SLAM_ALIGN_MAX_RMSE_M", 2.5)),
        "freshness_s": max(0.2, f("SLAM_ALIGN_FRESHNESS_S", 2.0)),
        "allow_scale": False,  # stereo default
    }


def _span_m(pairs: list[Pair]) -> float:
    if len(pairs) < 2:
        return 0.0
    xs = [p.ax for p in pairs]
    ys = [p.ay for p in pairs]
    dx = max(xs) - min(xs)
    dy = max(ys) - min(ys)
    return float((dx * dx + dy * dy) ** 0.5)


def _slam_span_3d(pairs: list[Pair]) -> float:
    """
    Rough size of the SLAM sample cloud (3D), useful to detect scale mismatches.
    """
    if len(pairs) < 2:
        return 0.0
    xs = [p.sx for p in pairs]
    ys = [p.sy for p in pairs]
    zs = [p.sz for p in pairs]
    dx = max(xs) - min(xs)
    dy = max(ys) - min(ys)
    dz = max(zs) - min(zs)
    return float((dx * dx + dy * dy + dz * dz) ** 0.5)


def _trim_outliers(pairs: list[Pair], t, keep_fraction: float, min_keep: int) -> list[Pair]:
    """
    Keep the lowest-residual pairs to reduce the impact of SLAM relocalization jumps / mismatched pairing.
    This is intentionally simple (no refactors / no SLAM internals).
    """
    keep_fraction = max(0.3, min(float(keep_fraction), 1.0))
    min_keep = max(3, int(min_keep))
    if len(pairs) <= min_keep:
        return pairs
    scored: list[tuple[float, Pair]] = []
    for p in pairs:
        try:
            x, y = t.apply_xyz(p.sx, p.sy, p.sz)
            dx = float(x) - p.ax
            dy = float(y) - p.ay
            scored.append(((dx * dx + dy * dy) ** 0.5, p))
        except Exception:
            continue
    if len(scored) <= min_keep:
        return pairs
    scored.sort(key=lambda it: it[0])
    k = max(min_keep, int(len(scored) * keep_fraction))
    return [p for (_d, p) in scored[:k]]


def _auto_align_worker() -> None:
    params = _auto_align_params()
    last_ax: float | None = None
    last_ay: float | None = None
    last_sx: float | None = None
    last_sy: float | None = None
    last_sz: float | None = None

    with _auto_align_lock:
        _auto_align_state.update({"running": True, "last_error": None, "skipped_time": 0, "skipped_yaw": 0, "skipped_step": 0})

    while not _auto_align_stop.is_set():
        try:
            udp = get_udp_stats()
            with _auto_align_lock:
                _auto_align_state["slam_udp_packets"] = int(udp.get("packets", 0) or 0)
                _auto_align_state["slam_last_packet_age_s"] = udp.get("last_packet_age_s")

            pose = _read_airsim_pose_raw()
            ts = float(pose.get("ts", 0.0) or 0.0)
            if ts <= 0.0 or (time.time() - ts) > params["freshness_s"]:
                time.sleep(params["interval_s"])
                continue

            slam_pose = _read_slam_udp_pose(freshness_s=params["freshness_s"])
            if slam_pose is None:
                with _auto_align_lock:
                    _auto_align_state["last_error"] = "No fresh SLAM UDP pose. Check sender/port/bind."
                time.sleep(params["interval_s"])
                continue

            sx, sy, sz, syaw, slam_ts = slam_pose
            ax = float(pose.get("x", 0.0))
            ay = float(pose.get("y", 0.0))
            with _auto_align_lock:
                # We have both streams; clear the stale "no pose" error if present.
                if _auto_align_state.get("last_error", "").startswith("No fresh SLAM UDP pose"):
                    _auto_align_state["last_error"] = None

            # -------- Temporal alignment (critical) --------
            # SLAM UDP arrival time and AirSim pose write time must match closely.
            # Even 0.5–2.0s mismatch at a few m/s produces 5–10m RMSE.
            import os

            # Default must be compatible with the AirSim pose publish rate (see AIRSIM_POSE_PUBLISH_S).
            max_dt_s = float(os.getenv("SLAM_ALIGN_MAX_TIME_S", "0.25"))
            if max_dt_s > 0:
                dt_s = abs(float(slam_ts) - float(ts))
                if dt_s > max_dt_s:
                    with _auto_align_lock:
                        _auto_align_state["skipped_time"] = int(_auto_align_state.get("skipped_time", 0) or 0) + 1
                    time.sleep(params["interval_s"])
                    continue

            # -------- Yaw sanity (helps reject mismatched pairing) --------
            yaw_thr = float(os.getenv("SLAM_ALIGN_MAX_YAW_DEG", "30.0"))
            ayaw = _airsim_yaw_deg(pose)
            if ayaw is not None and yaw_thr > 0:
                syaw_deg = _slam_yaw_deg(syaw)
                yaw_err = abs(_norm_deg(syaw_deg - float(ayaw)))
                if yaw_err > yaw_thr:
                    with _auto_align_lock:
                        _auto_align_state["skipped_yaw"] = int(_auto_align_state.get("skipped_yaw", 0) or 0) + 1
                    time.sleep(params["interval_s"])
                    continue

            if (
                last_ax is not None
                and last_ay is not None
                and last_sx is not None
                and last_sy is not None
                and last_sz is not None
            ):
                da = ((ax - last_ax) ** 2 + (ay - last_ay) ** 2) ** 0.5
                # Use full 3D delta in SLAM coordinates so we reject jumps on any axis.
                ds = ((sx - last_sx) ** 2 + (sy - last_sy) ** 2 + (sz - last_sz) ** 2) ** 0.5
                if da < params["min_step_m"] or ds < params["min_step_m"]:
                    with _auto_align_lock:
                        _auto_align_state["skipped_step"] = int(_auto_align_state.get("skipped_step", 0) or 0) + 1
                    time.sleep(params["interval_s"])
                    continue
                if ds > params["max_step_m"]:
                    # Likely SLAM jump/relocalization.
                    time.sleep(params["interval_s"])
                    continue

            pairs = load_pairs()
            pairs.append(Pair(sx=sx, sy=sy, sz=sz, ax=ax, ay=ay))
            if len(pairs) > params["max_pairs"]:
                pairs = pairs[-params["max_pairs"] :]
            save_pairs(pairs)

            with _auto_align_lock:
                _auto_align_state["pairs_collected"] = len(pairs)

            last_ax, last_ay, last_sx, last_sy, last_sz = ax, ay, sx, sy, sz

            if len(pairs) >= params["min_pairs_to_solve"] and _span_m(pairs) >= params["min_span_m"]:
                # Lock axes for stability (default x,z).
                best_t = None
                best_e = None
                axes = _slam_axes_locked()
                try:
                    t0 = solve_transform(pairs, allow_scale=bool(params.get("allow_scale")), slam_axes=axes)  # type: ignore[arg-type]
                    e0 = rmse_m(t0, pairs)
                    best_t, best_e = t0, e0
                except Exception:
                    pass

                if best_t is not None and best_e is not None:
                    # Optional robust step: trim outliers (helps when SLAM occasionally jumps or
                    # when pairing is slightly time-misaligned during fast motion).
                    try:
                        import os

                        keep_frac = float(os.getenv("SLAM_ALIGN_TRIM_FRACTION", "0.8"))
                        min_keep = int(os.getenv("SLAM_ALIGN_TRIM_MIN_KEEP", "12"))
                        trimmed = _trim_outliers(pairs, best_t, keep_fraction=keep_frac, min_keep=min_keep)
                        if len(trimmed) >= max(params["min_pairs_to_solve"], 3) and len(trimmed) < len(pairs):
                            t_trim = solve_transform(trimmed, allow_scale=bool(params.get("allow_scale")), slam_axes=axes)  # type: ignore[arg-type]
                            e_trim = rmse_m(t_trim, trimmed)
                            # Compare on full pair set to avoid overfitting the trimmed subset.
                            e_full = rmse_m(t_trim, pairs)
                            if e_full < best_e:
                                best_t, best_e = t_trim, e_full
                    except Exception:
                        pass

                    with _auto_align_lock:
                        _auto_align_state["last_attempt_rmse_m"] = float(best_e)
                        _auto_align_state["last_attempt_axes"] = list(best_t.slam_axes)
                        _auto_align_state["last_attempt_model"] = best_t.model
                        _auto_align_history.append(
                            {
                                "ts": time.time(),
                                "pairs": len(pairs),
                                "span_m": _span_m(pairs),
                                "slam_span_m": _slam_span_3d(pairs),
                                "rmse_m": float(best_e),
                                "axes": list(best_t.slam_axes),
                                "model": best_t.model,
                                "accepted": bool(best_e <= params["max_rmse_m"]),
                            }
                        )
                        if len(_auto_align_history) > _AUTO_ALIGN_HISTORY_MAX:
                            del _auto_align_history[: len(_auto_align_history) - _AUTO_ALIGN_HISTORY_MAX]
                    # Always compute/store the best transform when possible, but use RMSE
                    # to limit what actions it can drive (click-to-go gating), not whether it exists.
                    # To avoid "thrashing" between good/bad solutions, only overwrite an existing
                    # transform if it improves the current RMSE on the latest pair set.
                    t_prev = load_transform()
                    prev_rmse = rmse_m(t_prev, pairs) if t_prev is not None else float("inf")
                    if t_prev is None or best_e < (prev_rmse - 1e-6):
                        now = time.time()
                        save_transform(
                            best_t,
                            pairs_used=len(pairs),
                            rmse_m=float(best_e),
                            quality=_rmse_quality(float(best_e)) or "unknown",
                            ts=now,
                        )
                        with _auto_align_lock:
                            _auto_align_state["last_solve_rmse_m"] = float(best_e)
                            _auto_align_state["last_solve_ts"] = now
                            _auto_align_state["last_error"] = None
        except Exception as e:
            with _auto_align_lock:
                _auto_align_state["last_error"] = f"{type(e).__name__}: {e}"
                _auto_align_history.append({"ts": time.time(), "error": _auto_align_state["last_error"]})
                if len(_auto_align_history) > _AUTO_ALIGN_HISTORY_MAX:
                    del _auto_align_history[: len(_auto_align_history) - _AUTO_ALIGN_HISTORY_MAX]

        time.sleep(params["interval_s"])

    with _auto_align_lock:
        _auto_align_state["running"] = False


@router.post("/align/auto/start")
def align_auto_start():
    global _auto_align_thread
    if _auto_align_thread and _auto_align_thread.is_alive():
        return {"ok": True, "running": True}
    _auto_align_stop.clear()
    _auto_align_thread = threading.Thread(target=_auto_align_worker, daemon=True)
    _auto_align_thread.start()
    return {"ok": True, "running": True, "params": _auto_align_params()}


@router.post("/align/auto/stop")
def align_auto_stop():
    _auto_align_stop.set()
    return {"ok": True, "running": False}


@router.get("/align/auto/status")
def align_auto_status():
    with _auto_align_lock:
        st = dict(_auto_align_state)
    # Include UDP status snapshot for debugging (why pairs/RMSE may be stuck).
    st["udp"] = get_udp_stats()
    return {"ok": True, **st, "params": _auto_align_params()}

@router.get("/align/auto/history")
def align_auto_history(limit: int = 300):
    """
    Return recent auto-align RMSE attempts for visualization.
    """
    limit = max(10, min(int(limit or 300), _AUTO_ALIGN_HISTORY_MAX))
    with _auto_align_lock:
        return {"ok": True, "items": _auto_align_history[-limit:]}

@router.post("/align/auto/history/clear")
def align_auto_history_clear():
    with _auto_align_lock:
        _auto_align_history.clear()
    return {"ok": True}


@router.get("/align/status")
def align_status():
    pairs = load_pairs()
    t = load_transform()
    out = {"ok": True, "pairs": len(pairs), "has_transform": bool(t)}
    if t is not None:
        e = rmse_m(t, pairs)
        payload = load_transform_payload()
        out.update(
            {
                "model": t.model,
                "scale": t.scale,
                "R": [[t.r11, t.r12], [t.r21, t.r22]],
                "t": [t.tx, t.ty],
                "slam_axes": [t.slam_axes[0], t.slam_axes[1]],
                "rmse_m": e,
                "quality": _rmse_quality(e),
                # UX: tell the frontend how much authority to grant SLAM-based clicks.
                "goto_max_step_m": _rmse_goto_limit_m(e),
                # Persisted metadata (may be older than current pairs RMSE).
                "saved_rmse_m": payload.get("rmse_m"),
                "saved_quality": payload.get("quality"),
                "saved_ts": payload.get("ts"),
            }
        )
    return out


@router.post("/align/reset")
def align_reset():
    reset_pairs()
    return {"ok": True, "msg": "alignment pairs reset"}


@router.post("/align/reset_all")
def align_reset_all():
    reset_pairs()
    reset_transform()
    return {"ok": True, "msg": "alignment pairs + transform reset"}


def _airsim_speed_xy_mps() -> float | None:
    """
    Best-effort speed estimate from the published AirSim pose file.
    Requires drone_motion >= current version (writes v_xy).
    """
    pose = _read_airsim_pose_raw()
    try:
        vxy = pose.get("v_xy")
        if vxy is None:
            vx = float(pose.get("vx"))
            vy = float(pose.get("vy"))
            return float((vx * vx + vy * vy) ** 0.5)
        return float(vxy)
    except Exception:
        return None


def _read_slam_pose_for_snap(freshness_s: float) -> tuple[float, float, float, float, float] | None:
    """
    For stop-and-snap calibration we only need a *fresh* pose.
    We accept both sender timestamps and arrival timestamps here.
    Returns (sx,sy,sz,yaw,ts).
    """
    udp = get_udp_stats()
    try:
        packets = int(udp.get("packets", 0) or 0)
    except Exception:
        packets = 0
    age = udp.get("last_packet_age_s")
    if packets <= 0 or age is None or float(age) > freshness_s:
        return None
    try:
        sx = float(latest_pose.get("x", 0.0))
        sy = float(latest_pose.get("y", 0.0))
        sz = float(latest_pose.get("z", 0.0))
        yaw = float(latest_pose.get("yaw", 0.0))
        ts = float(latest_pose.get("ts", 0.0) or 0.0)
        if ts <= 0.0:
            ts = time.time()
        return (sx, sy, sz, yaw, ts)
    except Exception:
        return None


@router.post("/align/snap")
def align_snap():
    """
    Capture a calibration pair at the *current* pose:
      SLAM (latest_pose.x, latest_pose.z)  <->  AirSim (x,y)
    """
    # Stop-and-snap: require hover (low speed) to reduce pairing/motion bias.
    vxy = _airsim_speed_xy_mps()
    try:
        max_vxy = float(os.getenv("SLAM_SNAP_MAX_VXY_MPS", "0.25"))
    except Exception:
        max_vxy = 0.25
    if bool(_snap_wizard.get("enabled")):
        try:
            max_vxy = float(_snap_wizard.get("snap_max_vxy_mps", max_vxy))
        except Exception:
            pass
    if vxy is not None and vxy > max_vxy:
        raise HTTPException(status_code=409, detail=f"Refusing snap: drone not hovering (v_xy={vxy:.2f} m/s, max={max_vxy:.2f}).")

    freshness_s = float(_auto_align_params()["freshness_s"])
    if bool(_snap_wizard.get("enabled")):
        try:
            freshness_s = float(_snap_wizard.get("freshness_s", freshness_s))
        except Exception:
            pass
    slam_pose = _read_slam_pose_for_snap(freshness_s=freshness_s)
    if slam_pose is None:
        raise HTTPException(status_code=503, detail="No fresh SLAM UDP pose available.")
    sx, sy, sz, _syaw, slam_ts = slam_pose
    ax, ay = _read_airsim_xy()

    # Reject likely SLAM relocalization/map-reset jumps that would poison calibration.
    try:
        max_slam_jump_m = float(os.getenv("SLAM_SNAP_MAX_SLAM_JUMP_M", "3.0"))
    except Exception:
        max_slam_jump_m = 3.0
    try:
        max_airsim_move_m = float(os.getenv("SLAM_SNAP_MAX_AIRSIM_MOVE_M", "0.8"))
    except Exception:
        max_airsim_move_m = 0.8
    if bool(_snap_wizard.get("enabled")):
        try:
            max_slam_jump_m = float(_snap_wizard.get("snap_max_slam_jump_m", max_slam_jump_m))
        except Exception:
            pass
        try:
            max_airsim_move_m = float(_snap_wizard.get("snap_max_airsim_move_m", max_airsim_move_m))
        except Exception:
            pass

    pairs_existing = load_pairs()
    if pairs_existing:
        p_last = pairs_existing[-1]
        ds = ((float(sx) - float(p_last.sx)) ** 2 + (float(sy) - float(p_last.sy)) ** 2 + (float(sz) - float(p_last.sz)) ** 2) ** 0.5
        da = ((float(ax) - float(p_last.ax)) ** 2 + (float(ay) - float(p_last.ay)) ** 2) ** 0.5
        if ds > max_slam_jump_m and da < max_airsim_move_m:
            raise HTTPException(
                status_code=409,
                detail=f"Rejected snap: SLAM jump detected (ds={ds:.2f}m, da={da:.2f}m).",
            )

    pairs = append_pair(Pair(sx=sx, sy=sy, sz=sz, ax=ax, ay=ay))
    return {
        "ok": True,
        "pairs": len(pairs),
        "pair": {"sx": sx, "sy": sy, "sz": sz, "ax": ax, "ay": ay, "slam_ts": slam_ts, "source": "udp"},
        "airsim_v_xy_mps": vxy,
    }


@router.post("/align/solve")
def align_solve(allow_scale: bool = False):
    """
    Solve SLAM->AirSim transform from captured pairs.
    Stereo SLAM usually wants allow_scale=false (scale≈1).
    """
    pairs = load_pairs()
    t, e, used = solve_transform_robust(pairs, allow_scale=bool(allow_scale))
    now = time.time()
    save_transform(t, pairs_used=int(used), rmse_m=float(e), quality=_rmse_quality(float(e)) or "unknown", ts=now)
    return {
        "ok": True,
        "pairs_used": int(used),
        "pairs_total": len(pairs),
        "rmse_m": float(e),
        "model": t.model,
        "scale": float(t.scale),
        "slam_axes": list(t.slam_axes),
        "quality": _rmse_quality(float(e)),
    }


class SlamGotoReq(BaseModel):
    # SLAM pose coords. The ground plane axes are selected by the fitted transform.
    sx: float
    sy: float = 0.0
    sz: float = 0.0
    z: float


@router.post("/goto_slam")
def goto_slam(req: SlamGotoReq):
    """
    SLAM-based click-to-go:
    - Converts SLAM (sx,sz) -> AirSim (x,y) using the calibrated transform.
    - Sends a normal AirSim goto.
    """
    t = load_transform()
    if t is None:
        raise HTTPException(status_code=400, detail="No SLAM->AirSim transform. Use /slam/align/snap then /slam/align/solve.")
    pairs = load_pairs()
    e = rmse_m(t, pairs)
    max_step = _rmse_goto_limit_m(e)
    if max_step is not None and max_step <= 0.0:
        raise HTTPException(
            status_code=400,
            detail=f"SLAM->AirSim alignment quality is too poor for navigation (rmse={e:.2f}m). Move around, improve tracking, then re-solve.",
        )
    # Req provides full SLAM XYZ; transform chooses its ground-plane axes.
    x, y = t.apply_xyz(float(req.sx), float(req.sy), float(req.sz))

    # Gate authority based on RMSE: allow short-range nudges while calibration is improving.
    if max_step is not None:
        try:
            cx, cy = _read_airsim_xy()
            step = ((float(x) - cx) ** 2 + (float(y) - cy) ** 2) ** 0.5
            if step > max_step:
                raise HTTPException(
                    status_code=400,
                    detail=f"Alignment is not yet accurate enough for a {step:.1f}m move (rmse={e:.2f}m). Max allowed step is {max_step:.1f}m.",
                )
        except HTTPException:
            raise
        except Exception:
            # If we can't read current pose, be conservative when rmse is not great.
            raise HTTPException(
                status_code=503,
                detail="AirSim pose unavailable to gate SLAM click-to-go. Try again after the pose endpoint is healthy.",
            )

    send_goto(float(x), float(y), float(req.z))
    return {"ok": True, "target": {"x": float(x), "y": float(y), "z": float(req.z)}}

# ---------- SLAM MODE (mapping/localization) ----------
class SlamModeReq(BaseModel):
    mode: str  # "mapping" or "localization"

@router.post("/mode")
def set_slam_mode(req: SlamModeReq):
    if get_slam_state() == "calibrating":
        return JSONResponse(status_code=409, content={"ok": False, "error": "Calibration is running. Wait for it to finish."})
    if req.mode not in ("mapping", "localization"):
        raise HTTPException(status_code=400, detail="mode must be 'mapping' or 'localization'")
    send_slam_mode(req.mode)   # ✅ NOT drone anymore
    set_slam_state("mapping" if req.mode == "mapping" else "localization")
    return {"ok": True, "mode": req.mode, "slam_state": get_slam_state()}

# ---------- DRONE MODE (mapping/explore) ----------
class DroneModeReq(BaseModel):
    mode: str  # "mapping" or "explore"

@router.post("/drone/mode")
def set_drone_mode(req: DroneModeReq):
    send_drone_mode(req.mode)
    return {"ok": True, "mode": req.mode}

@router.post("/drone/stop")
def stop_drone_only():
    send_drone_mode("explore")   # ✅ IMPORTANT: explore, not localization
    send_hover()
    return {"ok": True, "msg": "Drone stopped, SLAM still running"}

@router.post("/drone/restart")
def restart_drone_motion():
    # Hard restart of drone_motion process (useful after code changes).
    stop_drone()
    start_drone()
    send_drone_mode("mapping")
    return {"ok": True, "msg": "Drone motion restarted"}
