import subprocess
import os
import signal
import sys
from pathlib import Path

DRONE_PROC = None
BASE_DIR = Path(__file__).resolve().parent.parent  # slam_web/
PID_FILE = BASE_DIR / "drone_motion.pid"

def _pid_is_running(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    else:
        return True

def start_drone():
    global DRONE_PROC
    if DRONE_PROC and DRONE_PROC.poll() is None:
        return
    if PID_FILE.exists():
        try:
            old_pid = int(PID_FILE.read_text().strip())
        except Exception:
            old_pid = None
        if old_pid and _pid_is_running(old_pid):
            return
        try:
            PID_FILE.unlink()
        except Exception:
            pass

    DRONE_PROC = subprocess.Popen(
        [sys.executable, "app/drone_motion.py"],
        cwd=str(BASE_DIR),
        preexec_fn=os.setsid
    )
    try:
        PID_FILE.write_text(str(DRONE_PROC.pid))
    except Exception:
        pass

def stop_drone():
    global DRONE_PROC
    pid = None
    if DRONE_PROC and DRONE_PROC.poll() is None:
        pid = DRONE_PROC.pid
    elif PID_FILE.exists():
        try:
            pid = int(PID_FILE.read_text().strip())
        except Exception:
            pid = None

    if pid:
        try:
            os.killpg(os.getpgid(pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        except Exception:
            try:
                os.kill(pid, signal.SIGTERM)
            except Exception:
                pass

    DRONE_PROC = None
    try:
        PID_FILE.unlink()
    except Exception:
        pass
