import airsim
import numpy as np
import cv2
import subprocess
import struct
import time
import os

# ----------------- CONFIG -----------------
ORB_SLAM_DIR = os.path.expanduser("~/ORB_SLAM3")
VOC = os.path.join(ORB_SLAM_DIR, "Vocabulary", "ORBvoc.txt")
CFG = os.path.join(ORB_SLAM_DIR, "live_configs", "Stereo-Airsim-LIVE.yaml")
TRAJ_OUT = os.path.join(ORB_SLAM_DIR, "CameraTrajectory_LIVE.txt")

BRIDGE_BIN = os.path.join(ORB_SLAM_DIR, "build", "live_stereo_bridge")

CAPTURE_RATE = 30.0
DT = 1.0 / CAPTURE_RATE
ALTITUDE = -10.0
FORWARD_CLEAR = 30.0
LEG_LENGTH = 20.0
SPEED = 1.5
# ------------------------------------------

def encode_png(img):
    ok, buf = cv2.imencode(".png", img)
    if not ok:
        return None
    return buf.tobytes()

def send_frame(proc, ts_ns, left_img, right_img):
    left_bytes = encode_png(left_img)
    right_bytes = encode_png(right_img)
    if left_bytes is None or right_bytes is None:
        print("[WARN] Failed to encode PNG")
        return

    # uint64 ts, uint32 left_len, uint32 right_len + bytes
    header = struct.pack("<QII", ts_ns, len(left_bytes), len(right_bytes))
    proc.stdin.write(header)
    proc.stdin.write(left_bytes)
    proc.stdin.write(right_bytes)
    proc.stdin.flush()

def main():
    # 1) Start the C++ bridge
    proc = subprocess.Popen(
        [BRIDGE_BIN, VOC, CFG, TRAJ_OUT],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=0
    )

    # Optional: print bridge logs in background
    # but simplest: just read asynchronously in another thread if needed.
    # For now we'll ignore to keep script simple.

    # 2) AirSim connection
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)

    client.takeoffAsync().join()
    client.moveToZAsync(ALTITUDE, 2).join()
    print(f"Reached altitude {abs(ALTITUDE)} m")

    client.moveByVelocityBodyFrameAsync(SPEED, 0, 0, FORWARD_CLEAR / SPEED).join()
    print("Moved forward for clearance...")
    time.sleep(2)

    frame_id = 0
    t_start = time.time()

    def grab_stereo():
        left_raw = client.simGetImage("StereoCameraLeft", airsim.ImageType.Scene)
        right_raw = client.simGetImage("StereoCameraRight", airsim.ImageType.Scene)

        if left_raw is None or right_raw is None:
            print("[WARN] Empty image from AirSim")
            return None, None

        left_arr = np.frombuffer(left_raw, np.uint8)
        right_arr = np.frombuffer(right_raw, np.uint8)

        left_img = cv2.imdecode(left_arr, cv2.IMREAD_COLOR)
        right_img = cv2.imdecode(right_arr, cv2.IMREAD_COLOR)
        return left_img, right_img

    # 3) Fly square while streaming to SLAM
    print("Starting live mapping loop...")
    try:
        for leg in range(4):
            duration = LEG_LENGTH / SPEED
            t_leg_start = time.time()

            client.moveByVelocityBodyFrameAsync(
                SPEED, 0, 0, duration,
                drivetrain=airsim.DrivetrainType.ForwardOnly,
                yaw_mode=airsim.YawMode(False, 0)
            )

            while time.time() - t_leg_start < duration:
                left_img, right_img = grab_stereo()
                if left_img is None or right_img is None:
                    time.sleep(DT)
                    continue

                # EuRoC-like timestamp in ns (relative)
                t_now = time.time() - t_start
                ts_ns = int(t_now * 1e9)

                send_frame(proc, ts_ns, left_img, right_img)
                frame_id += 1
                time.sleep(DT)

            # Rotate 90° for next leg, also streaming
            rotate_duration = 2.0
            yaw_rate = 45.0  # deg/s
            client.rotateByYawRateAsync(yaw_rate, rotate_duration)
            t_rot_start = time.time()
            while time.time() - t_rot_start < rotate_duration:
                left_img, right_img = grab_stereo()
                if left_img is None or right_img is None:
                    time.sleep(DT)
                    continue

                t_now = time.time() - t_start
                ts_ns = int(t_now * 1e9)
                send_frame(proc, ts_ns, left_img, right_img)
                frame_id += 1
                time.sleep(DT)

            client.hoverAsync().join()
            time.sleep(0.5)

    finally:
        print("Flight done, sending sentinel to SLAM...")
        # Sentinel: timestamp=0, sizes=0
        if proc.stdin:
            proc.stdin.write(struct.pack("<QII", 0, 0, 0))
            proc.stdin.flush()
            proc.stdin.close()

        # Land the drone
        client.hoverAsync().join()
        client.landAsync().join()
        client.armDisarm(False)
        client.enableApiControl(False)

        # Wait for bridge to finish saving map/trajectory
        proc.wait()
        print("SLAM bridge exited with code", proc.returncode)
        print("Live mapping session finished.")

if __name__ == "__main__":
    main()
