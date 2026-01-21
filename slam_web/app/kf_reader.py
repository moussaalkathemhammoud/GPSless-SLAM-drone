from pathlib import Path

ORB_SLAM3_ROOT = Path.home() / "ORB_SLAM3"
KF_FILE = ORB_SLAM3_ROOT / "live_keyframes.tum"

def read_keyframes(max_points: int = 5000):
    """
    Reads TUM file lines:
    timestamp tx ty tz qx qy qz qw
    Returns list of [x, y, z]
    """
    if not KF_FILE.exists():
        return []

    pts = []
    try:
        lines = KF_FILE.read_text().strip().splitlines()
        for line in lines[-max_points:]:
            if not line.strip():
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            # parts: t tx ty tz qx qy qz qw
            tx = float(parts[1])
            ty = float(parts[2])
            tz = float(parts[3])
            pts.append([tx, ty, tz])
    except Exception:
        return []

    return pts
