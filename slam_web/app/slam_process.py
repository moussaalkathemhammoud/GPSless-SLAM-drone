import subprocess
import signal
import os
import time
import sys
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent  # ~/slam_web

slam_process = None

ORB_SLAM3_ROOT = os.path.expanduser("~/ORB_SLAM3")

SLAM_CMD = [
    "./Examples_old/Stereo-Inertial/airsim_stereo_bridge",
    "Vocabulary/ORBvoc.txt",
    "Examples_old/Stereo-Inertial/Stereo-Airsim.yaml"
]

LOG_PATH = os.path.join(ORB_SLAM3_ROOT, "slam.log")

motion_process = None

PYTHON = os.path.expanduser("~/slam_web/venv/bin/python")

MOTION_CMD = [
    PYTHON,
    "app/drone_motion.py"
]
MOTION_LOG = str(BASE_DIR / "motion.log")


def is_running():
    global slam_process
    return slam_process is not None and slam_process.poll() is None

def start_slam():
    global slam_process, motion_process

    if slam_process is not None:
        return {"ok": False, "msg": "SLAM already running"}

    slam_process = subprocess.Popen(
        SLAM_CMD,
        cwd=ORB_SLAM3_ROOT,
        preexec_fn=os.setsid
    )

    # Give SLAM time to initialize cameras
    time.sleep(3)
    motion_log_f = open(MOTION_LOG, "a")
    motion_process = subprocess.Popen(
        [PYTHON, "app/drone_motion.py"],
        cwd=str(BASE_DIR),
        stdout=motion_log_f,
        stderr=motion_log_f,
        preexec_fn=os.setsid
    )

    return {
        "ok": True,
        "msg": "SLAM + motion started",
        "slam_pid": slam_process.pid,
        "motion_pid": motion_process.pid
    }


def stop_slam():
    global slam_process, motion_process

    if motion_process:
        os.killpg(os.getpgid(motion_process.pid), signal.SIGINT)
        motion_process = None

    if slam_process:
        os.killpg(os.getpgid(slam_process.pid), signal.SIGINT)
        slam_process = None

    return {"ok": True, "msg": "SLAM + motion stopped"}

def slam_status():
    return {
        "running": is_running(),
        "pid": None if slam_process is None else slam_process.pid,
        "log": LOG_PATH
    }
