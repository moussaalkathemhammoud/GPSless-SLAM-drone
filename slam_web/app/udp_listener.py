import socket
import threading
import math
import time
from typing import Any
import os

UDP_BIND_IP = os.getenv("SLAM_UDP_BIND_IP", "127.0.0.1")
try:
    UDP_PORT = int(os.getenv("SLAM_UDP_PORT", "5005"))
except Exception:
    UDP_PORT = 5005
try:
    # Multiply incoming SLAM positions by this factor to convert to meters.
    # Example: if your SLAM sender outputs centimeters, set SLAM_POSE_SCALE=0.01
    SLAM_POSE_SCALE = float(os.getenv("SLAM_POSE_SCALE", "1.0"))
except Exception:
    SLAM_POSE_SCALE = 1.0

latest_pose = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "yaw": 0.0,
    # Local arrival time of the most recent UDP pose (epoch seconds).
    # Used to time-align SLAM pose with AirSim pose samples.
    "ts": 0.0,
    # "sender" if the UDP payload includes a timestamp, else "arrival".
    "ts_source": "arrival",
    # Optional raw timestamp value included by the sender (may be non-epoch).
    "ts_sender": None,
}

_traj_lock = threading.Lock()
_trajectory_segments: list[list[list[float]]] = [[]]
_last_traj_point: tuple[float, float, float] | None = None
_udp_packets = 0
_last_udp_time = 0.0
_bind_error: str | None = None
_listener_started = False
_listener_thread: threading.Thread | None = None

TRAJ_MIN_STEP_M = 0.10   # ignore tiny jitter
TRAJ_MAX_JUMP_M = 5.0    # treat as relocalization/reset -> new segment
TRAJ_MAX_POINTS = 5000   # total points across segments

# 🔴 THIS WAS MISSING
last_kf = None
KF_DISTANCE = 0.15  # meters (tune 0.2–0.5)

def is_new_keyframe(x, y, z):
    global last_kf
    if last_kf is None:
        last_kf = (x, y, z)
        return True
    dx = x - last_kf[0]
    dy = y - last_kf[1]
    dz = z - last_kf[2]
    dist = (dx*dx + dy*dy + dz*dz) ** 0.5
    if dist > KF_DISTANCE:
        last_kf = (x, y, z)
        return True
    return False


def _append_trajectory_point(x: float, y: float, z: float) -> None:
    global _last_traj_point, _trajectory_segments
    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
        return

    with _traj_lock:
        # Defensive: never allow the segments container to be empty.
        # If it ever becomes empty (e.g., after trimming), re-seed it.
        if not _trajectory_segments:
            _trajectory_segments = [[]]

        if _last_traj_point is not None:
            dx = x - _last_traj_point[0]
            dy = y - _last_traj_point[1]
            dz = z - _last_traj_point[2]
            d = (dx * dx + dy * dy + dz * dz) ** 0.5
            if d < TRAJ_MIN_STEP_M:
                return
            if d > TRAJ_MAX_JUMP_M:
                # Relocalization/reset: start a new polyline segment.
                if _trajectory_segments and _trajectory_segments[-1]:
                    _trajectory_segments.append([])

        _trajectory_segments[-1].append([x, y, z])
        _last_traj_point = (x, y, z)

        # Trim to last TRAJ_MAX_POINTS points (across segments).
        total = sum(len(seg) for seg in _trajectory_segments)
        if total <= TRAJ_MAX_POINTS:
            return

        to_drop = total - TRAJ_MAX_POINTS
        while to_drop > 0 and _trajectory_segments:
            seg = _trajectory_segments[0]
            if len(seg) <= to_drop:
                to_drop -= len(seg)
                _trajectory_segments.pop(0)
            else:
                del seg[:to_drop]
                to_drop = 0

        # Remove leading empties if any.
        while _trajectory_segments and not _trajectory_segments[0]:
            _trajectory_segments.pop(0)
        if not _trajectory_segments:
            _trajectory_segments = [[]]


def get_trajectory_segments() -> list[list[list[float]]]:
    with _traj_lock:
        return [seg[:] for seg in _trajectory_segments if seg]

def get_udp_stats() -> dict[str, Any]:
    with _traj_lock:
        total_points = sum(len(seg) for seg in _trajectory_segments)
        segments = len([seg for seg in _trajectory_segments if seg])
    age_s = None
    if _last_udp_time:
        age_s = time.time() - _last_udp_time
    return {
        "udp_port": UDP_PORT,
        "udp_bind_ip": UDP_BIND_IP,
        "listener_started": _listener_started,
        "bind_error": _bind_error,
        "packets": _udp_packets,
        "last_packet_age_s": age_s,
        "latest_pose": latest_pose,
        "segments": segments,
        "points": total_points,
        "min_step_m": TRAJ_MIN_STEP_M,
        "max_jump_m": TRAJ_MAX_JUMP_M,
    }



def udp_loop():
    global _udp_packets, _last_udp_time
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    except Exception:
        pass
    try:
        sock.bind((UDP_BIND_IP, UDP_PORT))
    except Exception as e:
        global _bind_error
        _bind_error = f"{type(e).__name__}: {e}"
        return

    while True:
        data, _ = sock.recvfrom(1024)
        try:
            parts = list(map(float, data.decode().split()))
            if len(parts) < 4:
                continue
            x, y, z, yaw = parts[:4]
            ts_source = "arrival"
            ts_sender = None
            if len(parts) >= 5:
                ts_sender = float(parts[4])
                # Heuristic: sender timestamps should be "epoch seconds" (order 1e9).
                # If the sender timestamp is NOT epoch-like (e.g. seconds since start),
                # we keep ts_source="arrival" and use local arrival time for alignment.
                if ts_sender > 1e8:
                    ts_source = "sender"
                    slam_ts = ts_sender
                else:
                    slam_ts = time.time()
            else:
                slam_ts = time.time()
            if SLAM_POSE_SCALE != 1.0:
                x *= SLAM_POSE_SCALE
                y *= SLAM_POSE_SCALE
                z *= SLAM_POSE_SCALE
            latest_pose.update(
                {
                    "x": x,
                    "y": y,
                    "z": z,
                    "yaw": yaw,
                    "ts": float(slam_ts),
                    "ts_source": ts_source,
                    "ts_sender": ts_sender,
                }
            )
            _append_trajectory_point(x, y, z)
            _udp_packets += 1
            _last_udp_time = time.time()
        except:
            pass

def start_udp_listener():
    global _listener_started, _listener_thread
    if _listener_started and _listener_thread and _listener_thread.is_alive():
        return
    _listener_started = True
    _listener_thread = threading.Thread(target=udp_loop, daemon=True)
    _listener_thread.start()
