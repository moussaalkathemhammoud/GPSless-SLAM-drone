from __future__ import annotations

"""
static_geo_init.py

One-time initializer for the fake GPS transform.

IMPORTANT NOTE (AirSim frame):
AirSim's MultirotorState.position is typically reported in a LOCAL NED frame whose
origin is the vehicle's "home"/spawn. That means the drone usually starts at
(x=0,y=0) even if you move the pawn's spawn in the Unreal world.

Therefore, to align the image map with the simulator spawn you usually want:
- offset_x_m/offset_y_m = 0
- lat0/lng0 chosen such that it corresponds to the spawn point on your image

This module simply ensures `slam_web/calibration.json` exists and does NOT
overwrite it if you've already configured it.
"""

import json
import os
from pathlib import Path

from .gps_utils import get_fake_gps_origin, get_fake_gps_yaw_deg


def init_static_origin_at_spawn() -> None:
    """
    If FAKE_GPS_STATIC_ORIGIN_AT_SPAWN=1 (default), ensure calibration.json exists.
    It is only created if missing, to avoid "snapping back" after you configure it.
    """
    enabled = os.getenv("FAKE_GPS_STATIC_ORIGIN_AT_SPAWN", "1").lower() not in ("0", "false", "no")
    if not enabled:
        return

    base_dir = Path(__file__).resolve().parent.parent  # slam_web/
    calib_file = base_dir / "calibration.json"
    if calib_file.exists():
        return

    origin = get_fake_gps_origin()
    yaw_deg = get_fake_gps_yaw_deg()

    payload = {
        "lat0": float(origin.lat0),
        "lng0": float(origin.lng0),
        "yaw_deg": float(yaw_deg),
        # Default: align fake GPS origin to AirSim local origin (spawn/home).
        "offset_x_m": 0.0,
        "offset_y_m": 0.0,
    }
    try:
        calib_file.write_text(json.dumps(payload, indent=2))
    except Exception:
        pass
