import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print("Listening for SLAM poses...")

while True:
    data, addr = sock.recvfrom(1024)
    msg = data.decode().strip()
    try:
        x, y, z, yaw, map_x, map_y = map(float, msg.split())
        print(
            f"x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.2f} | "
            f"map_x={map_x:.1f}, map_y={map_y:.1f}"
        )
    except Exception as e:
        print("Malformed:", msg)
