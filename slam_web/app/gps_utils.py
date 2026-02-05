"""
gps_utils.py

Fake (local) GPS <-> AirSim XY conversion utilities.

ASSUMPTIONS (kept intentionally simple / deterministic):
- AirSim world is local NED in meters.
  - +x = North (forward)
  - +y = East  (right)
- We geo-reference the AirSim ground plane (x=0,y=0) to a fixed "fake GPS" origin.
- Conversion uses an equirectangular approximation (good for small areas: ~km scale).

Frontend must send ONLY lat/lng (never raw AirSim coordinates) for missions.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
from typing import Iterable, Tuple

EARTH_RADIUS_M = 6378137.0  # WGS84 semi-major axis (meters)

def _load_map_config_file() -> dict:
    """
    Optional static map configuration overrides (additive).
    File location: <repo_root>/static/map/config.json

    This is intentionally separate from calibration.json:
    - calibration.json is interactive/experimental
    - static/map/config.json is deterministic "deployment" config
    """
    try:
        # gps_utils.py lives in: <repo_root>/slam_web/app/gps_utils.py
        repo_root = Path(__file__).resolve().parents[2]
        cfg_file = repo_root / "static" / "map" / "config.json"
        if not cfg_file.exists():
            return {}

        import json

        data = json.loads(cfg_file.read_text())
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def _load_calibration_file() -> dict:
    """
    Optional persistent calibration overrides (additive).
    File location: slam_web/calibration.json
    """
    try:
        calib_file = Path(__file__).resolve().parent.parent / "calibration.json"  # slam_web/calibration.json
        if not calib_file.exists():
            return {}

        import json

        data = json.loads(calib_file.read_text())
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


@dataclass(frozen=True)
class FakeGpsOrigin:
    lat0: float
    lng0: float


def get_fake_gps_origin() -> FakeGpsOrigin:
    """
    Fixed origin for the local, geo-referenced "fake GPS" map.

    Override via env vars to align with your chosen map imagery location:
      - FAKE_GPS_LAT0
      - FAKE_GPS_LNG0
    """
    calib = _load_calibration_file()
    lat0 = float(calib.get("lat0", os.getenv("FAKE_GPS_LAT0", "37.4219999")))
    lng0 = float(calib.get("lng0", os.getenv("FAKE_GPS_LNG0", "-122.0840575")))
    return FakeGpsOrigin(lat0=lat0, lng0=lng0)


def get_fake_gps_yaw_deg() -> float:
    """
    Rotation used to align AirSim XY with the geo-referenced map.

    Convention:
      - We rotate the AirSim (x,y) vector by +yaw_deg (CCW) to obtain (north,east).
      - yaw_deg = 0 means: +x increases latitude (north), +y increases longitude (east).
    """
    calib = _load_calibration_file()
    try:
        return float(calib.get("yaw_deg", os.getenv("FAKE_GPS_YAW_DEG", "0.0")))
    except Exception:
        return 0.0


def get_fake_gps_offset_xy_m() -> tuple[float, float]:
    """
    Translation between AirSim XY and the fake GPS origin.

    Default is (0,0) (AirSim origin is also the map origin).

    If your AirSim environment's "spawn/home" is NOT at (0,0) in the coordinates
    reported by `getMultirotorState()`, set it deterministically in:
      static/map/config.json:
        { "origin_airsim_xy_m": { "x": <meters>, "y": <meters> } }

    This keeps the frontend free of raw AirSim numbers while still aligning the
    image map to the AirSim world consistently.
    """
    cfg = _load_map_config_file()
    o = cfg.get("origin_airsim_xy_m") if isinstance(cfg, dict) else None
    try:
        if isinstance(o, dict):
            x = float(o.get("x", 0.0))
            y = float(o.get("y", 0.0))
            return (x, y)
    except Exception:
        pass
    return (0.0, 0.0)


def _rotate_xy(x: float, y: float, deg_ccw: float) -> tuple[float, float]:
    a = math.radians(deg_ccw)
    ca = math.cos(a)
    sa = math.sin(a)
    return (ca * x - sa * y, sa * x + ca * y)


def latlng_to_airsim_xy(
    lat: float,
    lng: float,
    origin: FakeGpsOrigin,
    yaw_deg: float = 0.0,
    offset_xy_m: tuple[float, float] = (0.0, 0.0),
) -> Tuple[float, float]:
    """
    Convert lat/lng (degrees) -> AirSim (x,y) in meters.
      - returns (x_north_m, y_east_m)
    """
    lat0_rad = math.radians(origin.lat0)
    dlat_rad = math.radians(lat - origin.lat0)
    dlng_rad = math.radians(lng - origin.lng0)

    north_m = dlat_rad * EARTH_RADIUS_M
    east_m = dlng_rad * EARTH_RADIUS_M * math.cos(lat0_rad)

    # Inverse of airsim->NE rotation: rotate by -yaw to get AirSim XY.
    x_airsim, y_airsim = _rotate_xy(north_m, east_m, -yaw_deg)
    return (x_airsim + offset_xy_m[0], y_airsim + offset_xy_m[1])


def airsim_xy_to_latlng(
    x_airsim_m: float,
    y_airsim_m: float,
    origin: FakeGpsOrigin,
    yaw_deg: float = 0.0,
    offset_xy_m: tuple[float, float] = (0.0, 0.0),
) -> Tuple[float, float]:
    """
    Convert AirSim (x,y) in meters -> lat/lng (degrees).
    """
    lat0_rad = math.radians(origin.lat0)

    # Subtract translation, then rotate AirSim XY into (north,east) for geo conversion.
    x0 = x_airsim_m - offset_xy_m[0]
    y0 = y_airsim_m - offset_xy_m[1]
    north_m, east_m = _rotate_xy(x0, y0, yaw_deg)

    dlat_rad = north_m / EARTH_RADIUS_M
    # Guard against cos(lat0)=0 at poles; not expected in this project.
    cos_lat0 = math.cos(lat0_rad) or 1e-12
    dlng_rad = east_m / (EARTH_RADIUS_M * cos_lat0)

    lat = origin.lat0 + math.degrees(dlat_rad)
    lng = origin.lng0 + math.degrees(dlng_rad)
    return (lat, lng)


def latlngs_to_airsim_xy(
    latlngs: Iterable[Tuple[float, float]],
    origin: FakeGpsOrigin,
    yaw_deg: float = 0.0,
    offset_xy_m: tuple[float, float] = (0.0, 0.0),
) -> list[Tuple[float, float]]:
    return [
        latlng_to_airsim_xy(lat, lng, origin, yaw_deg=yaw_deg, offset_xy_m=offset_xy_m)
        for (lat, lng) in latlngs
    ]


def airsim_xy_to_latlngs(
    xys: Iterable[Tuple[float, float]],
    origin: FakeGpsOrigin,
    yaw_deg: float = 0.0,
    offset_xy_m: tuple[float, float] = (0.0, 0.0),
) -> list[Tuple[float, float]]:
    return [airsim_xy_to_latlng(x, y, origin, yaw_deg=yaw_deg, offset_xy_m=offset_xy_m) for (x, y) in xys]
