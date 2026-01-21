import airsim
import time
import signal
import sys
import threading

ALTITUDE = -10.0
SPEED = 1.0
LEG_LENGTH = 25.0
HOVER_TIME = 0.5

running = True

def signal_handler(sig, frame):
    global running
    running = False
    print("\n🛑 Stop signal received", flush=True)

signal.signal(signal.SIGINT, signal_handler)

def run_motion():
    print("🚁 Connecting to AirSim...", flush=True)
    client = airsim.MultirotorClient(ip="172.20.144.1")
    client.confirmConnection()

    print("✅ Connected to AirSim", flush=True)
    client.enableApiControl(True)
    client.armDisarm(True)

    client.takeoffAsync().join()
    client.moveToZAsync(ALTITUDE, 2).join()
    print("✅ Takeoff complete", flush=True)

    time.sleep(2)

    def move_forward(dist):
        duration = dist / SPEED
        client.moveByVelocityBodyFrameAsync(
            SPEED, 0, 0,
            duration,
            drivetrain=airsim.DrivetrainType.ForwardOnly,
            yaw_mode=airsim.YawMode(False, 0)
        ).join()
        time.sleep(HOVER_TIME)

    def rotate_yaw(deg):
        rate = 45.0 if deg > 0 else -45.0
        duration = abs(deg) / 45.0
        client.rotateByYawRateAsync(rate, duration).join()
        time.sleep(HOVER_TIME)

    print("🚁 LIVE SLAM MAPPING STARTED", flush=True)

    try:
        while running:
            for _ in range(4):
                move_forward(LEG_LENGTH)
                rotate_yaw(90)

            rotate_yaw(180)
            move_forward(LEG_LENGTH)
            rotate_yaw(-180)
            move_forward(LEG_LENGTH)

    finally:
        print("🧹 Cleaning up", flush=True)
        client.hoverAsync().join()
        client.landAsync().join()
        client.armDisarm(False)
        client.enableApiControl(False)
        print("🏁 Drone landed", flush=True)
client = None
client_lock = threading.Lock()

def get_client():
    global client
    if client is None:
        client = airsim.MultirotorClient(ip="172.20.144.1")
        client.confirmConnection()
        client.enableApiControl(True)
        client.armDisarm(True)
    return client

def goto_target(x, y, z):
    def worker():
        c = get_client()
        print(f"✈️ Going to ({x:.2f}, {y:.2f}, {z:.2f})")
        c.moveToPositionAsync(
            x, y, z,
            velocity=1.5,
            drivetrain=airsim.DrivetrainType.ForwardOnly,
            yaw_mode=airsim.YawMode(False, 0)
        ).join()

    threading.Thread(target=worker, daemon=True).start()

if __name__ == "__main__":
    run_motion()
