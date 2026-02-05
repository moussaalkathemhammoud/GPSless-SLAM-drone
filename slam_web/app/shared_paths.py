from __future__ import annotations

from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent  # slam_web/
MISSIONS_DIR = BASE_DIR / "missions"
AIRSIM_POSE_FILE = MISSIONS_DIR / "airsim_pose.json"

