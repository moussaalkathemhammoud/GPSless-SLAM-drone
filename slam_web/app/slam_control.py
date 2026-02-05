# app/slam_control.py
import socket

SLAM_CMD_IP = "127.0.0.1"
SLAM_CMD_PORT = 6007

def send_slam_mode(mode: str):
    # mode: "mapping" | "localization"
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(mode.encode("utf-8"), (SLAM_CMD_IP, SLAM_CMD_PORT))
