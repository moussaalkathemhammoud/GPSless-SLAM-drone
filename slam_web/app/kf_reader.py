from pathlib import Path
import math

ORB_SLAM3_ROOT = Path.home() / "ORB_SLAM3"
KF_FILE = ORB_SLAM3_ROOT / "live_keyframes.tum"

_base: tuple[float, float, float, float] | None = None  # (t0, tx0, ty0, tz0)

def _read_first_keyframe():
    if not KF_FILE.exists():
        return None
    try:
        for line in KF_FILE.read_text().splitlines():
            if not line.strip():
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            t0 = float(parts[0])
            tx0 = float(parts[1])
            ty0 = float(parts[2])
            tz0 = float(parts[3])
            if not (math.isfinite(t0) and math.isfinite(tx0) and math.isfinite(ty0) and math.isfinite(tz0)):
                continue
            return (t0, tx0, ty0, tz0)
    except Exception:
        return None
    return None

def read_keyframes(max_points: int = 5000, min_step_m: float = 0.05, max_jump_m: float = 1e9):
    """
    Reads TUM file lines:
    timestamp tx ty tz qx qy qz qw
    Returns list of [x, y, z]
    """
    if not KF_FILE.exists():
        return []

    pts = []
    last = None
    try:
        global _base
        first = _read_first_keyframe()
        # If SLAM restarted (file rewritten from a new run), reset our base.
        if first is not None and (_base is None or abs(first[0] - _base[0]) > 1e-3):
            _base = first

        lines = KF_FILE.read_text().strip().splitlines()
        for line in lines[-max_points:]:
            if not line.strip():
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            # parts: t tx ty tz qx qy qz qw
            t = float(parts[0])
            tx = float(parts[1])
            ty = float(parts[2])
            tz = float(parts[3])

            if not (math.isfinite(t) and math.isfinite(tx) and math.isfinite(ty) and math.isfinite(tz)):
                continue

            if _base is None:
                _base = (t, tx, ty, tz)

            # Return coordinates relative to a stable base (first keyframe of the run).
            tx -= _base[1]
            ty -= _base[2]
            tz -= _base[3]

            if last is not None:
                dx = tx - last[0]
                dy = ty - last[1]
                dz = tz - last[2]
                d = (dx * dx + dy * dy + dz * dz) ** 0.5
                if d < min_step_m:
                    continue
                if d > max_jump_m:
                    # Drop SLAM outliers/resets to avoid long "spikes" on the map.
                    continue

            pts.append([tx, ty, tz])
            last = (tx, ty, tz)
    except Exception:
        return []

    return pts
