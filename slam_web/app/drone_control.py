import socket, json

DRONE_CMD_IP = "127.0.0.1"
DRONE_CMD_PORT = 6006

def _send(msg: dict):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(json.dumps(msg).encode("utf-8"), (DRONE_CMD_IP, DRONE_CMD_PORT))

def send_hover():
    _send({"type": "hover"})

def send_drone_mode(mode: str):
    _send({"type": "mode", "mode": mode})

def send_goto(x: float, y: float, z: float, velocity: float | None = None):
    msg = {"type": "goto", "x": x, "y": y, "z": z}
    if velocity is not None:
        msg["velocity"] = float(velocity)
    _send(msg)

# ---------- Mission control (dynamic mapping) ----------
# NOTE: we intentionally keep the drone_motion process long-lived.
# Mission commands must never terminate the process; they only start/abort mission execution.

def send_mission_start(mission_id: str, mission_file: str):
    _send({"type": "mission_start", "mission_id": mission_id, "mission_file": mission_file})

def send_mission_abort(mission_id: str):
    _send({"type": "mission_abort", "mission_id": mission_id})


# ---------- Calibration motion (SLAM alignment) ----------
def send_calib_run(
    *,
    side_m: float,
    z: float,
    speed_mps: float,
    repeats: int = 3,
    pause_s: float = 0.7,
    yaw_seq_deg: list[float] | None = None,
    dist_seq_m: list[float] | None = None,
):
    _send(
        {
            "type": "calib_run",
            "side_m": float(side_m),
            "z": float(z),
            "speed_mps": float(speed_mps),
            "repeats": int(repeats),
            "pause_s": float(pause_s),
            "yaw_seq_deg": yaw_seq_deg if yaw_seq_deg is not None else [0.0, 90.0, 180.0, 270.0],
            "dist_seq_m": dist_seq_m,
        }
    )


def send_calib_stop():
    _send({"type": "calib_stop"})


def send_calib_static_run(
    *,
    side_m: float,
    z: float,
    speed_mps: float,
    laps: int = 2,
    hover_s: float = 0.6,
):
    _send(
        {
            "type": "calib_static_run",
            "side_m": float(side_m),
            "z": float(z),
            "speed_mps": float(speed_mps),
            "laps": int(laps),
            "hover_s": float(hover_s),
        }
    )


def send_forward(*, dist_m: float, speed_mps: float = 1.0, z: float | None = None):
    msg: dict = {"type": "forward", "dist_m": float(dist_m), "speed_mps": float(speed_mps)}
    if z is not None:
        msg["z"] = float(z)
    _send(msg)
