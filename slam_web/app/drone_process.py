import subprocess
import time

drone_proc = None

def start_drone():
    global drone_proc

    if drone_proc:
        return False

    # Give AirSim time to be ready
    time.sleep(3)

    drone_proc = subprocess.Popen(
        ["python3", "app/drone_motion.py"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    return True

def stop_drone():
    global drone_proc

    if drone_proc:
        drone_proc.terminate()
        drone_proc = None
