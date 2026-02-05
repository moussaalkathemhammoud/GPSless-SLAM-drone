from __future__ import annotations

"""
airsim_pose.py

Small helper to read the drone's *actual* AirSim pose for visualization and missions.
This avoids using SLAM/ORB pose for the geo map, which can be in a different frame/scale.

This module is intentionally standalone (does not import drone_motion.py) to avoid
side effects and keep changes isolated.
"""

import math
import os
import threading
from typing import Any, Optional

import airsim

AIRSIM_DRONE_IP = os.getenv("AIRSIM_DRONE_IP", "172.20.144.1")

_tls = threading.local()


def _get_client() -> airsim.MultirotorClient:
    """
    AirSim's Python client uses msgpack-rpc (tornado IOLoop) under the hood.
    Sharing a single client across multiple FastAPI worker threads can trigger:
      RuntimeError: IOLoop is already running

    To keep this isolated and stable, we keep one client per thread (thread-local).
    """
    c: Optional[airsim.MultirotorClient] = getattr(_tls, "client", None)
    if c is None:
        c = airsim.MultirotorClient(ip=AIRSIM_DRONE_IP)
        c.confirmConnection()
        _tls.client = c
    return c


def get_airsim_pose() -> dict[str, Any]:
    """
    Returns AirSim NED pose in meters:
      - x: forward/north
      - y: right/east
      - z: down (negative is up)
    """
    c = _get_client()
    s = c.getMultirotorState()
    p = s.kinematics_estimated.position
    o = s.kinematics_estimated.orientation
    _, _, yaw = airsim.to_eularian_angles(o)
    yaw_deg = (math.degrees(yaw) % 360.0 + 360.0) % 360.0

    try:
        ci = c.simGetCollisionInfo()
        has_collided = bool(ci.has_collided)
        collision_ts = float(ci.time_stamp or 0.0)
    except Exception:
        has_collided = False
        collision_ts = 0.0

    return {
        "x": float(p.x_val),
        "y": float(p.y_val),
        "z": float(p.z_val),
        "yaw_deg": float(yaw_deg),
        "has_collided": has_collided,
        "collision_time_stamp": collision_ts,
    }
