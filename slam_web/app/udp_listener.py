import socket
import threading
import math
import time

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

latest_pose = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "yaw": 0.0
}

trajectory = []

# 🔴 THIS WAS MISSING
last_kf = None
KF_DISTANCE = 0.15  # meters (tune 0.2–0.5)

def is_new_keyframe(x, y, z):
    global last_kf
    if last_kf is None:
        last_kf = (x, y, z)
        return True
    dx = x - last_kf[0]
    dy = y - last_kf[1]
    dz = z - last_kf[2]
    dist = (dx*dx + dy*dy + dz*dz) ** 0.5
    if dist > KF_DISTANCE:
        last_kf = (x, y, z)
        return True
    return False



def udp_loop():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    while True:
        data, _ = sock.recvfrom(1024)
        try:
            parts = list(map(float, data.decode().split()))
            if len(parts) < 4:
                continue
            x, y, z, yaw = parts[:4]
            latest_pose.update({"x": x, "y": y, "z": z, "yaw": yaw})
        except:
            pass

def start_udp_listener():
    threading.Thread(target=udp_loop, daemon=True).start()