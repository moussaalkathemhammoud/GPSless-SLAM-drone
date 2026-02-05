import airsim
import json
import math
import os
import signal
import socket
import threading
import time
from pathlib import Path
from typing import Optional

# ============================================================
# drone_motion.py
#
# Long-lived AirSim motion controller used by slam_web.
#
# Supports:
# - Static mapping loop: forward legs + arc 90° turns (MODE="mapping")
# - Explore mode: external goto/hover commands (MODE="explore")
# - Calibration-only motion primitives (triggered via UDP commands):
#     * "calib_run"         (rotate + forward legs)
#     * "calib_static_run"  (static-mapping-like: forward + arc turns + hover)
# - Thread-safe AirSim RPC usage (serialize all client calls).
# - Pose publishing to slam_web/missions/airsim_pose.json for FastAPI pairing.
# ============================================================

# ---------------- Config ----------------
DRONE_IP = os.getenv("AIRSIM_IP", "172.20.144.1")

DRONE_CMD_IP = os.getenv("DRONE_CMD_IP", "127.0.0.1")
DRONE_CMD_PORT = int(os.getenv("DRONE_CMD_PORT", "6006"))

# NED: negative is up.
ALTITUDE = float(os.getenv("DRONE_ALTITUDE_NED", "-10.0"))
SAFE_MIN_ALTITUDE = float(os.getenv("DRONE_SAFE_MIN_ALT_NED", "-2.0"))

# Static mapping defaults (can be overridden by env).
SPEED = float(os.getenv("MAPPING_SPEED_MPS", "1.0"))
LEG_LENGTH = float(os.getenv("MAPPING_LEG_M", "23.0"))
HOVER_TIME = float(os.getenv("MAPPING_HOVER_S", "0.3"))

CONTROL_DT = float(os.getenv("MAPPING_CONTROL_DT_S", "0.25"))
LEG_TIMEOUT_S = float(os.getenv("MAPPING_LEG_TIMEOUT_S", "60.0"))

TURN_FWD_SPEED = float(os.getenv("MAPPING_TURN_FWD_MPS", "0.6"))
TURN_YAW_RATE_MAX = float(os.getenv("MAPPING_TURN_YAW_MAX_DPS", "25.0"))
TURN_YAW_RATE_MIN = float(os.getenv("MAPPING_TURN_YAW_MIN_DPS", "8.0"))
TURN_KP = float(os.getenv("MAPPING_TURN_KP", "0.6"))
TURN_TOL_DEG = float(os.getenv("MAPPING_TURN_TOL_DEG", "2.0"))
TURN_TIMEOUT_S = float(os.getenv("MAPPING_TURN_TIMEOUT_S", "8.0"))

# ---------------- Global state ----------------
MODE = "mapping"  # "mapping" | "explore"
running = True

client: Optional[airsim.MultirotorClient] = None
client_lock = threading.Lock()
motion_lock = threading.Lock()
rpc_lock = threading.Lock()

# Calibration motion status file (read by FastAPI).
_BASE_DIR = Path(__file__).resolve().parent.parent  # slam_web/
_POSE_FILE = _BASE_DIR / "missions" / "airsim_pose.json"
_SPAWN_FILE = _BASE_DIR / "missions" / "airsim_spawn_xy.json"
_CALIB_STATUS_FILE = _BASE_DIR / "missions" / "calib_motion_status.json"

calib_abort_evt = threading.Event()
calib_active_evt = threading.Event()

# Mission execution (optional; used by mission_routes).
mission_abort_evt = threading.Event()
mission_thread: Optional[threading.Thread] = None
mission_active_id: Optional[str] = None


def _mission_is_active() -> bool:
    return bool(mission_thread and mission_thread.is_alive())


def _write_json(path: Path, payload: dict) -> None:
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload))
    except Exception:
        pass


def _write_calib_status(payload: dict) -> None:
    try:
        d = dict(payload)
        d["ts"] = time.time()
        _write_json(_CALIB_STATUS_FILE, d)
    except Exception:
        pass


def _get_client() -> airsim.MultirotorClient:
    global client
    with client_lock:
        if client is None:
            c = airsim.MultirotorClient(ip=DRONE_IP)
            with rpc_lock:
                c.confirmConnection()
                c.enableApiControl(True)
                c.armDisarm(True)
            client = c
        return client


def _get_position_xy(c: airsim.MultirotorClient) -> tuple[float, float]:
    with rpc_lock:
        p = c.getMultirotorState().kinematics_estimated.position
    return (float(p.x_val), float(p.y_val))


def _get_yaw_deg(c: airsim.MultirotorClient) -> float:
    with rpc_lock:
        o = c.getMultirotorState().kinematics_estimated.orientation
    _, _, yaw = airsim.to_eularian_angles(o)
    return (math.degrees(float(yaw)) % 360.0 + 360.0) % 360.0


def _norm_deg(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def stop_and_hover(c: airsim.MultirotorClient) -> None:
    try:
        with rpc_lock:
            c.moveByVelocityAsync(0, 0, 0, 0.2).join()
    except Exception:
        pass
    try:
        with rpc_lock:
            c.hoverAsync().join()
    except Exception:
        pass


def cancel_motion() -> None:
    c = _get_client()
    try:
        with rpc_lock:
            c.cancelLastTask()
    except Exception:
        pass
    stop_and_hover(c)


def set_mode(new_mode: str) -> None:
    global MODE
    m = str(new_mode or "").strip().lower()
    if m not in ("mapping", "explore"):
        return
    # Don't let mapping fight active missions/calibration.
    if m == "mapping" and (_mission_is_active() or calib_active_evt.is_set()):
        return
    MODE = m


def move_forward_locked_alt(c: airsim.MultirotorClient, v_mps: float, z_ned: float, dt_s: float) -> None:
    with rpc_lock:
        c.moveByVelocityZBodyFrameAsync(
            float(v_mps),
            0.0,
            float(z_ned),
            float(dt_s),
            drivetrain=airsim.DrivetrainType.ForwardOnly,
            yaw_mode=airsim.YawMode(True, 0.0),
        ).join()


def arc_turn_right_90(c: airsim.MultirotorClient, z_ref: float) -> None:
    """
    Approximate a 90° right turn as a smooth arc (forward + yaw-rate).
    """
    yaw0 = _get_yaw_deg(c)
    t0 = time.time()
    while running and (time.time() - t0) < TURN_TIMEOUT_S and (MODE == "mapping" or calib_active_evt.is_set()) and not calib_abort_evt.is_set():
        yaw = _get_yaw_deg(c)
        delta = (yaw - yaw0 + 360.0) % 360.0  # right turn increases yaw
        remaining = 90.0 - delta
        if remaining <= TURN_TOL_DEG:
            break
        yaw_rate = max(TURN_YAW_RATE_MIN, min(TURN_YAW_RATE_MAX, abs(remaining) * TURN_KP))
        try:
            with rpc_lock:
                c.moveByVelocityZBodyFrameAsync(
                    float(TURN_FWD_SPEED),
                    0.0,
                    float(z_ref),
                    float(CONTROL_DT),
                    drivetrain=airsim.DrivetrainType.ForwardOnly,
                    yaw_mode=airsim.YawMode(False, float(yaw_rate)),
                ).join()
        except Exception:
            break
    stop_and_hover(c)


def goto_target(x: float, y: float, z: float, velocity: float = 1.2) -> None:
    if MODE != "explore":
        return

    def worker():
        with motion_lock:
            c = _get_client()
            target_z = float(z)
            if target_z > SAFE_MIN_ALTITUDE:
                target_z = SAFE_MIN_ALTITUDE
            # Face travel direction (yaw) for better stability.
            try:
                cx, cy = _get_position_xy(c)
                yaw_deg = _norm_deg(math.degrees(math.atan2(float(y) - cy, float(x) - cx)))
            except Exception:
                yaw_deg = 0.0
            try:
                with rpc_lock:
                    c.rotateToYawAsync(float(yaw_deg), margin=2.0).join()
            except Exception:
                pass
            with rpc_lock:
                c.moveToPositionAsync(
                    float(x),
                    float(y),
                    float(target_z),
                    velocity=float(max(0.2, min(float(velocity), 3.0))),
                    drivetrain=airsim.DrivetrainType.ForwardOnly,
                    yaw_mode=airsim.YawMode(False, float(yaw_deg)),
                ).join()
            stop_and_hover(c)

    threading.Thread(target=worker, daemon=True).start()


def _publish_pose_loop() -> None:
    """
    Publish AirSim pose to a file for FastAPI (thread-safe; avoids RPC from API threads).
    """
    c = _get_client()
    # Save spawn once.
    try:
        x0, y0 = _get_position_xy(c)
        _write_json(_SPAWN_FILE, {"x": float(x0), "y": float(y0)})
    except Exception:
        pass

    def interval_s() -> float:
        try:
            v = float(os.getenv("AIRSIM_POSE_PUBLISH_S", "0.05"))
        except Exception:
            v = 0.05
        return max(0.05, min(v, 1.0))

    while running:
        try:
            with rpc_lock:
                s = c.getMultirotorState()
            p = s.kinematics_estimated.position
            v = s.kinematics_estimated.linear_velocity
            o = s.kinematics_estimated.orientation
            _, _, yaw = airsim.to_eularian_angles(o)
            yaw_deg = (math.degrees(float(yaw)) % 360.0 + 360.0) % 360.0
            vxy = float((float(v.x_val) ** 2 + float(v.y_val) ** 2) ** 0.5)
            _write_json(
                _POSE_FILE,
                {
                    "x": float(p.x_val),
                    "y": float(p.y_val),
                    "z": float(p.z_val),
                    "vx": float(v.x_val),
                    "vy": float(v.y_val),
                    "vz": float(v.z_val),
                    "v_xy": float(vxy),
                    "yaw_deg": float(yaw_deg),
                    "ts": time.time(),
                },
            )
        except Exception:
            pass
        time.sleep(interval_s())


def _calib_rotate_to_yaw(c: airsim.MultirotorClient, yaw_deg: float) -> None:
    try:
        with rpc_lock:
            c.rotateToYawAsync(float(yaw_deg), margin=2.0).join()
    except Exception:
        # fallback: proportional yaw rate
        t0 = time.time()
        while running and not calib_abort_evt.is_set() and (time.time() - t0) < 6.0:
            yaw = _get_yaw_deg(c)
            err = _norm_deg(float(yaw_deg) - yaw)
            if abs(err) <= 2.0:
                break
            rate = max(10.0, min(60.0, abs(err) * 0.8))
            rate = rate if err > 0 else -rate
            try:
                with rpc_lock:
                    c.rotateByYawRateAsync(float(rate), 0.2).join()
            except Exception:
                break


def _calib_square_motion(
    c: airsim.MultirotorClient,
    *,
    side_m: float,
    z_ned: float,
    speed_mps: float,
    repeats: int,
    pause_s: float,
    yaw_seq_deg: list[float],
    dist_seq_m: list[float] | None = None,
) -> None:
    side_m = max(1.0, float(side_m))
    z_ned = float(z_ned)
    speed_mps = max(0.2, min(float(speed_mps), 2.0))
    repeats = max(1, int(repeats))
    pause_s = max(0.0, min(float(pause_s), 3.0))
    yaw_seq_deg = [float(y) for y in (yaw_seq_deg or [0.0, 90.0, 180.0, 270.0])]
    if dist_seq_m is None or len(dist_seq_m) != len(yaw_seq_deg):
        dist_seq_m = [float(side_m) for _ in yaw_seq_deg]
    else:
        dist_seq_m = [float(d) for d in dist_seq_m]

    try:
        with rpc_lock:
            c.takeoffAsync().join()
    except Exception:
        pass
    try:
        with rpc_lock:
            c.moveToZAsync(z_ned, max(0.5, speed_mps)).join()
    except Exception:
        pass

    _write_calib_status({"running": True, "phase": "start"})
    for rep in range(repeats):
        if not running or calib_abort_evt.is_set():
            break
        for yaw_deg, dist_m in zip(yaw_seq_deg, dist_seq_m):
            if not running or calib_abort_evt.is_set():
                break
            _write_calib_status({"running": True, "phase": "rotate", "yaw_deg": float(yaw_deg), "repeat": rep + 1})
            _calib_rotate_to_yaw(c, float(yaw_deg))
            stop_and_hover(c)
            if pause_s:
                time.sleep(pause_s)

            duration_s = max(0.2, float(dist_m)) / float(speed_mps)
            t0 = time.time()
            _write_calib_status({"running": True, "phase": "forward", "yaw_deg": float(yaw_deg), "dist_m": float(dist_m), "repeat": rep + 1})
            while running and not calib_abort_evt.is_set() and (time.time() - t0) < duration_s:
                try:
                    move_forward_locked_alt(c, float(speed_mps), float(z_ned), 0.2)
                except Exception:
                    break
            stop_and_hover(c)
            if pause_s:
                time.sleep(pause_s)

    _write_calib_status({"running": False, "phase": "done", "aborted": bool(calib_abort_evt.is_set())})


def _calib_static_arc_motion(
    c: airsim.MultirotorClient,
    *,
    side_m: float,
    z_ned: float,
    speed_mps: float,
    laps: int,
    hover_s: float,
) -> None:
    """
    Calibration-only motion that mimics static mapping turns (arc turns).
    The backend should capture pairs only when phase == "hover".
    """
    side_m = max(1.0, float(side_m))
    z_ned = float(z_ned)
    speed_mps = max(0.2, min(float(speed_mps), 2.0))
    laps = max(1, int(laps))
    hover_s = max(0.0, min(float(hover_s), 3.0))

    try:
        with rpc_lock:
            c.takeoffAsync().join()
    except Exception:
        pass
    try:
        with rpc_lock:
            c.moveToZAsync(z_ned, max(0.5, speed_mps)).join()
    except Exception:
        pass

    _write_calib_status({"running": True, "phase": "start", "laps": laps})

    corner = 0
    for lap in range(laps):
        if not running or calib_abort_evt.is_set():
            break
        for leg in range(4):
            if not running or calib_abort_evt.is_set():
                break
            _write_calib_status({"running": True, "phase": "leg_forward", "lap": lap + 1, "leg": leg + 1, "side_m": float(side_m)})

            # Forward leg by progress along heading (same idea as static mapping).
            yaw_deg = _get_yaw_deg(c)
            yaw_rad = math.radians(yaw_deg)
            hx, hy = math.cos(yaw_rad), math.sin(yaw_rad)
            sx0, sy0 = _get_position_xy(c)
            progress = 0.0
            t0 = time.time()
            while running and not calib_abort_evt.is_set():
                move_forward_locked_alt(c, float(speed_mps), float(z_ned), float(CONTROL_DT))
                cx, cy = _get_position_xy(c)
                dx, dy = cx - sx0, cy - sy0
                s = dx * hx + dy * hy
                progress = max(progress, s)
                if progress >= side_m:
                    break
                if time.time() - t0 > LEG_TIMEOUT_S:
                    break

            stop_and_hover(c)
            corner += 1
            _write_calib_status({"running": True, "phase": "hover", "corner": corner, "lap": lap + 1, "leg": leg + 1})
            if hover_s:
                time.sleep(hover_s)

            if progress < 0.5:
                _write_calib_status({"running": True, "phase": "warn_no_progress", "corner": corner})
                break

            _write_calib_status({"running": True, "phase": "turn_arc", "corner": corner, "lap": lap + 1, "leg": leg + 1})
            arc_turn_right_90(c, float(z_ned))
            stop_and_hover(c)

    _write_calib_status({"running": False, "phase": "done", "aborted": bool(calib_abort_evt.is_set())})


def _run_mission_from_file(path: str) -> None:
    global mission_thread
    if mission_thread and mission_thread.is_alive():
        return

    def worker():
        global mission_active_id
        try:
            mission_abort_evt.clear()
            p = Path(path)
            data = json.loads(p.read_text())
            pts = data.get("waypoints_xyz") or data.get("waypoints") or data.get("points") or []
            if not isinstance(pts, list):
                return
            # If loop is enabled, repeat waypoints until mission_abort is requested.
            loop = bool(data.get("loop", False))
            set_mode("explore")
            c = _get_client()
            with rpc_lock:
                c.takeoffAsync().join()
                c.moveToZAsync(float(data.get("z_ned", ALTITUDE)), 2).join()

            try:
                default_v = float(os.getenv("MISSION_VELOCITY_MPS", "1.0"))
            except Exception:
                default_v = 1.0
            default_v = max(0.2, min(default_v, 3.0))

            try:
                default_hover_s = float(os.getenv("MISSION_WP_HOVER_S", "0.25"))
            except Exception:
                default_hover_s = 0.25
            default_hover_s = max(0.0, min(default_hover_s, 5.0))

            def _run_one_pass() -> None:
                for wp in pts:
                    if mission_abort_evt.is_set() or not running:
                        break
                    # Support both formats:
                    # - list/tuple: [x,y,z]
                    # - dict: {"x":..,"y":..,"z":..,"velocity":..,"hover_s":..}
                    if isinstance(wp, (list, tuple)) and len(wp) >= 3:
                        x, y, z = float(wp[0]), float(wp[1]), float(wp[2])
                        v = default_v
                        hover_s = default_hover_s
                    elif isinstance(wp, dict):
                        x = float(wp.get("x", 0.0))
                        y = float(wp.get("y", 0.0))
                        z = float(wp.get("z", ALTITUDE))
                        v = float(wp.get("velocity", default_v))
                        hover_s = float(wp.get("hover_s", default_hover_s))
                    else:
                        continue

                    v = max(0.2, min(float(v), 3.0))
                    hover_s = max(0.0, min(float(hover_s), 5.0))

                    # Face travel direction for better stability (same idea as click-to-go).
                    try:
                        cx, cy = _get_position_xy(c)
                        yaw_deg = _norm_deg(math.degrees(math.atan2(float(y) - cy, float(x) - cx)))
                        with rpc_lock:
                            c.rotateToYawAsync(float(yaw_deg), margin=2.0).join()
                    except Exception:
                        pass

                    with rpc_lock:
                        c.moveToPositionAsync(
                            float(x),
                            float(y),
                            float(z),
                            velocity=float(v),
                            drivetrain=airsim.DrivetrainType.ForwardOnly,
                            yaw_mode=airsim.YawMode(False, 0.0),
                        ).join()
                    stop_and_hover(c)
                    if hover_s:
                        time.sleep(hover_s)

            # Default behavior is one pass. Dynamic missions can set loop=true to run continuously.
            if loop:
                while running and not mission_abort_evt.is_set():
                    _run_one_pass()
            else:
                _run_one_pass()
        except Exception:
            pass
        finally:
            mission_active_id = None
            mission_abort_evt.clear()

    mission_thread = threading.Thread(target=worker, daemon=True)
    mission_thread.start()


def _command_server() -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((DRONE_CMD_IP, DRONE_CMD_PORT))
    sock.setblocking(False)

    while running:
        try:
            data, _addr = sock.recvfrom(65535)
        except BlockingIOError:
            time.sleep(0.05)
            continue
        try:
            msg = json.loads(data.decode("utf-8"))
            t = msg.get("type")

            if t == "mode":
                set_mode(msg.get("mode", "mapping"))

            elif t == "hover":
                set_mode("explore")
                cancel_motion()

            elif t == "goto":
                goto_target(
                    float(msg.get("x", 0.0)),
                    float(msg.get("y", 0.0)),
                    float(msg.get("z", ALTITUDE)),
                    velocity=float(msg.get("velocity", 1.2)),
                )

            elif t == "forward":
                def _worker(payload: dict):
                    with motion_lock:
                        try:
                            set_mode("explore")
                            cancel_motion()
                            c = _get_client()

                            dist_m = float(payload.get("dist_m", 0.0))
                            speed_mps = float(payload.get("speed_mps", 1.0))
                            dist_m = max(0.0, min(dist_m, 200.0))
                            speed_mps = max(0.2, min(speed_mps, 3.0))

                            z_ned = float(payload.get("z", ALTITUDE))
                            if z_ned > SAFE_MIN_ALTITUDE:
                                z_ned = SAFE_MIN_ALTITUDE

                            try:
                                with rpc_lock:
                                    c.takeoffAsync().join()
                            except Exception:
                                pass
                            try:
                                with rpc_lock:
                                    c.moveToZAsync(float(z_ned), max(0.5, speed_mps)).join()
                            except Exception:
                                pass

                            duration_s = float(dist_m) / float(speed_mps) if speed_mps > 0 else 0.0
                            t0 = time.time()
                            while running and (time.time() - t0) < duration_s:
                                try:
                                    move_forward_locked_alt(c, float(speed_mps), float(z_ned), float(CONTROL_DT))
                                except Exception:
                                    break
                            stop_and_hover(c)
                        except Exception:
                            pass

                threading.Thread(target=_worker, args=(msg,), daemon=True).start()

            elif t == "mission_start":
                mid = str(msg.get("mission_id", "") or "")
                mfile = str(msg.get("mission_file", ""))
                if mid:
                    global mission_active_id
                    mission_active_id = mid
                if mfile:
                    _run_mission_from_file(mfile)

            elif t == "mission_abort":
                mid = str(msg.get("mission_id", "") or "")
                # If the caller provides an id, only abort that mission.
                if (not mid) or (mission_active_id is None) or (mid == mission_active_id):
                    mission_abort_evt.set()

            elif t == "calib_stop":
                calib_abort_evt.set()
                _write_calib_status({"running": False, "phase": "stop"})

            elif t == "calib_run":
                if calib_active_evt.is_set():
                    continue
                calib_abort_evt.clear()
                calib_active_evt.set()

                def _worker(payload: dict):
                    with motion_lock:
                        try:
                            set_mode("explore")
                            cancel_motion()
                            c = _get_client()
                            yaw_seq = payload.get("yaw_seq_deg") or [0.0, 90.0, 180.0, 270.0]
                            dist_seq = payload.get("dist_seq_m")
                            _calib_square_motion(
                                c,
                                side_m=float(payload.get("side_m", 8.0)),
                                z_ned=float(payload.get("z", ALTITUDE)),
                                speed_mps=float(payload.get("speed_mps", 0.7)),
                                repeats=int(payload.get("repeats", 1)),
                                pause_s=float(payload.get("pause_s", 0.7)),
                                yaw_seq_deg=[float(y) for y in yaw_seq],
                                dist_seq_m=None if not isinstance(dist_seq, list) else [float(d) for d in dist_seq],
                            )
                        finally:
                            calib_active_evt.clear()

                threading.Thread(target=_worker, args=(msg,), daemon=True).start()

            elif t == "calib_static_run":
                if calib_active_evt.is_set():
                    continue
                calib_abort_evt.clear()
                calib_active_evt.set()

                def _worker(payload: dict):
                    with motion_lock:
                        try:
                            set_mode("explore")
                            cancel_motion()
                            c = _get_client()
                            _calib_static_arc_motion(
                                c,
                                side_m=float(payload.get("side_m", 5.0)),
                                z_ned=float(payload.get("z", ALTITUDE)),
                                speed_mps=float(payload.get("speed_mps", 0.6)),
                                laps=int(payload.get("laps", 2)),
                                hover_s=float(payload.get("hover_s", 0.6)),
                            )
                        finally:
                            calib_active_evt.clear()

                threading.Thread(target=_worker, args=(msg,), daemon=True).start()

        except Exception:
            continue


def static_mapping() -> None:
    c = _get_client()
    cancel_motion()
    try:
        with rpc_lock:
            c.takeoffAsync().join()
            c.moveToZAsync(float(ALTITUDE), 2).join()
    except Exception:
        pass
    z_ref = float(ALTITUDE)

    while running:
        if MODE != "mapping":
            time.sleep(0.1)
            continue

        for _leg in range(4):
            if not running or MODE != "mapping":
                break

            yaw_deg = _get_yaw_deg(c)
            yaw_rad = math.radians(yaw_deg)
            hx, hy = math.cos(yaw_rad), math.sin(yaw_rad)

            sx0, sy0 = _get_position_xy(c)
            progress = 0.0
            t0 = time.time()

            while running and MODE == "mapping":
                move_forward_locked_alt(c, float(SPEED), float(z_ref), float(CONTROL_DT))
                cx, cy = _get_position_xy(c)
                dx, dy = cx - sx0, cy - sy0
                s = dx * hx + dy * hy
                progress = max(progress, s)
                if progress >= float(LEG_LENGTH):
                    break
                if time.time() - t0 > float(LEG_TIMEOUT_S):
                    break

            stop_and_hover(c)
            time.sleep(float(HOVER_TIME))

            if progress < 0.5:
                break
            arc_turn_right_90(c, float(z_ref))


def shutdown(sig, frame):
    global running
    running = False
    try:
        c = client
        if c is not None:
            with rpc_lock:
                c.hoverAsync().join()
                c.armDisarm(False)
                c.enableApiControl(False)
    except Exception:
        pass


def main() -> None:
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # Start background pose publisher + command server.
    threading.Thread(target=_publish_pose_loop, daemon=True).start()
    threading.Thread(target=_command_server, daemon=True).start()

    # Default static mapping loop.
    static_mapping()


if __name__ == "__main__":
    main()
