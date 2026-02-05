from __future__ import annotations

import json
import os
import threading
import time

from .shared_paths import AIRSIM_POSE_FILE
from .slam_airsim_alignment import Pair, load_pairs, save_pairs
from .udp_listener import latest_pose, get_udp_stats


_rec_thread: threading.Thread | None = None
_rec_stop = threading.Event()
_rec_lock = threading.Lock()
_rec_state: dict = {
    "running": False,
    "pairs_collected": 0,
    "last_error": None,
}


def _norm_deg(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def _slam_yaw_deg(yaw_val: float) -> float:
    import math

    y = float(yaw_val)
    if abs(y) <= (2.0 * math.pi + 0.5):
        y = math.degrees(y)
    return (y % 360.0 + 360.0) % 360.0


def _camera_offset_body_xyz() -> tuple[float, float, float]:
    def f(name: str, default: float) -> float:
        try:
            return float(os.getenv(name, str(default)))
        except Exception:
            return default

    return (f("SLAM_CAMERA_OFF_X_M", 0.25), f("SLAM_CAMERA_OFF_Y_M", 0.0), f("SLAM_CAMERA_OFF_Z_M", -0.30))


def _airsim_body_to_camera_xy(ax_body: float, ay_body: float, yaw_deg: float) -> tuple[float, float]:
    import math

    ox_b, oy_b, _oz_b = _camera_offset_body_xyz()
    a = math.radians(float(yaw_deg))
    ox = ox_b * math.cos(a) - oy_b * math.sin(a)
    oy = ox_b * math.sin(a) + oy_b * math.cos(a)
    return (float(ax_body) + ox, float(ay_body) + oy)


def _read_airsim_pose() -> tuple[float, float, float, float] | None:
    try:
        if not AIRSIM_POSE_FILE.exists():
            return None
        d = json.loads(AIRSIM_POSE_FILE.read_text())
        if not isinstance(d, dict):
            return None
        ts = float(d.get("ts", 0.0) or 0.0)
        if ts <= 0:
            return None
        ax = float(d.get("x", 0.0))
        ay = float(d.get("y", 0.0))
        yaw_deg = float(d.get("yaw_deg", d.get("yaw", 0.0)))
        return (ax, ay, yaw_deg, ts)
    except Exception:
        return None


def _read_slam_udp_pose(*, freshness_s: float) -> tuple[float, float, float, float, float] | None:
    """
    Returns (sx,sy,sz,yaw,ts) from SLAM UDP.
    If the sender does not provide an epoch timestamp, udp_listener uses local arrival
    time for ts (still epoch seconds) which can be aligned to the AirSim pose file ts.
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
        ts = float(latest_pose.get("ts", 0.0) or 0.0)
        if ts <= 0.0:
            return None
        sx = float(latest_pose.get("x", 0.0))
        sy = float(latest_pose.get("y", 0.0))
        sz = float(latest_pose.get("z", 0.0))
        yaw = float(latest_pose.get("yaw", 0.0))
        return (sx, sy, sz, yaw, ts)
    except Exception:
        return None


def start_mission_pair_recording() -> dict:
    global _rec_thread
    if _rec_thread and _rec_thread.is_alive():
        return {"ok": True, "running": True}

    _rec_stop.clear()

    def worker() -> None:
        last_ax: float | None = None
        last_ay: float | None = None
        last_sx: float | None = None
        last_sy: float | None = None
        last_sz: float | None = None

        def set_state(**kwargs) -> None:
            with _rec_lock:
                _rec_state.update(kwargs)

        interval_s = max(0.05, float(os.getenv("MISSION_PAIR_RECORD_INTERVAL_S", "0.25")))
        freshness_s = max(0.2, float(os.getenv("SLAM_ALIGN_FRESHNESS_S", "2.0")))
        max_dt_s = float(os.getenv("SLAM_ALIGN_MAX_TIME_S", "0.25"))
        min_step_m = max(0.05, float(os.getenv("SLAM_ALIGN_MIN_STEP_M", "0.5")))
        max_pairs = max(200, int(os.getenv("MISSION_PAIR_MAX_PAIRS", "2500")))
        yaw_thr = float(os.getenv("SLAM_ALIGN_MAX_YAW_DEG", "30.0"))

        set_state(running=True, last_error=None)
        try:
            while not _rec_stop.is_set():
                try:
                    air = _read_airsim_pose()
                    if air is None:
                        time.sleep(interval_s)
                        continue
                    ax_body, ay_body, ayaw_deg, air_ts = air
                    # Basic freshness check for AirSim pose file.
                    if (time.time() - float(air_ts)) > float(freshness_s):
                        time.sleep(interval_s)
                        continue

                    slam = _read_slam_udp_pose(freshness_s=freshness_s)
                    if slam is None:
                        set_state(last_error="No fresh SLAM UDP pose (sender timestamp required).")
                        time.sleep(interval_s)
                        continue
                    sx, sy, sz, syaw, slam_ts = slam
                    # Clear stale error once streams are healthy again.
                    with _rec_lock:
                        if str(_rec_state.get("last_error", "")).startswith("No fresh SLAM UDP pose"):
                            _rec_state["last_error"] = None

                    # Temporal alignment.
                    if max_dt_s > 0:
                        if abs(float(slam_ts) - float(air_ts)) > float(max_dt_s):
                            time.sleep(interval_s)
                            continue

                    # Yaw sanity.
                    if yaw_thr > 0:
                        yaw_err = abs(_norm_deg(_slam_yaw_deg(float(syaw)) - float(ayaw_deg)))
                        if yaw_err > float(yaw_thr):
                            time.sleep(interval_s)
                            continue

                    ax_cam, ay_cam = _airsim_body_to_camera_xy(ax_body, ay_body, ayaw_deg)

                    # Step gating (both streams must move).
                    if (
                        last_ax is not None
                        and last_ay is not None
                        and last_sx is not None
                        and last_sy is not None
                        and last_sz is not None
                    ):
                        da = ((ax_cam - last_ax) ** 2 + (ay_cam - last_ay) ** 2) ** 0.5
                        ds = ((sx - last_sx) ** 2 + (sy - last_sy) ** 2 + (sz - last_sz) ** 2) ** 0.5
                        if da < min_step_m or ds < min_step_m:
                            time.sleep(interval_s)
                            continue

                    pairs = load_pairs()
                    pairs.append(
                        Pair(
                            sx=float(sx),
                            sy=float(sy),
                            sz=float(sz),
                            ax=float(ax_cam),
                            ay=float(ay_cam),
                            ax_body=float(ax_body),
                            ay_body=float(ay_body),
                            yaw_deg=float(ayaw_deg),
                        )
                    )
                    if len(pairs) > max_pairs:
                        pairs = pairs[-max_pairs:]
                    save_pairs(pairs)
                    set_state(pairs_collected=len(pairs), last_error=None)

                    last_ax, last_ay, last_sx, last_sy, last_sz = ax_cam, ay_cam, sx, sy, sz
                except Exception as e:
                    set_state(last_error=f"{type(e).__name__}: {e}")
                    time.sleep(interval_s)
        finally:
            set_state(running=False)

    _rec_thread = threading.Thread(target=worker, daemon=True)
    _rec_thread.start()
    return {"ok": True, "running": True}


def stop_mission_pair_recording() -> dict:
    _rec_stop.set()
    with _rec_lock:
        _rec_state["running"] = False
    return {"ok": True, "running": False}


def mission_pair_recording_status() -> dict:
    with _rec_lock:
        return dict(_rec_state)
