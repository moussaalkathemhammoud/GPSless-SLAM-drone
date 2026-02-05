import subprocess
import signal
import os
import time
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
_slam_log_fh = None


def is_running():
    global slam_process
    return slam_process is not None and slam_process.poll() is None

def start_slam():
    global slam_process, _slam_log_fh

    if slam_process is not None:
        if slam_process.poll() is None:
            return {"ok": False, "msg": "SLAM already running"}
        # stale handle (process exited)
        slam_process = None

    os.makedirs(ORB_SLAM3_ROOT, exist_ok=True)
    _slam_log_fh = open(LOG_PATH, "a", buffering=1)
    slam_process = subprocess.Popen(
        SLAM_CMD,
        cwd=ORB_SLAM3_ROOT,
        stdout=_slam_log_fh,
        stderr=_slam_log_fh,
        preexec_fn=os.setsid
    )

    # Quick health check: if it exits immediately, surface it.
    time.sleep(0.2)
    rc = slam_process.poll()
    if rc is not None:
        pid = slam_process.pid
        slam_process = None
        return {
            "ok": False,
            "msg": f"SLAM failed to start (exit code {rc})",
            "slam_pid": pid,
            "log": LOG_PATH,
        }

    return {
        "ok": True,
        "msg": "SLAM started",
        "slam_pid": slam_process.pid,
        "log": LOG_PATH,
    }


def stop_slam():
    global slam_process, _slam_log_fh

    if slam_process:
        os.killpg(os.getpgid(slam_process.pid), signal.SIGINT)
        slam_process = None
    if _slam_log_fh:
        try:
            _slam_log_fh.close()
        finally:
            _slam_log_fh = None

    return {"ok": True, "msg": "SLAM stopped"}

def slam_status():
    running = slam_process is not None and slam_process.poll() is None
    returncode = None if running or slam_process is None else slam_process.poll()
    return {
        "running": running,
        "pid": None if slam_process is None else slam_process.pid,
        "returncode": returncode,
        "log": LOG_PATH
    }
