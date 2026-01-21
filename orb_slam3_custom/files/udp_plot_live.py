import socket
import matplotlib.pyplot as plt

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

xs, ys = [], []

plt.ion()
fig, ax = plt.subplots()
line, = ax.plot([], [], 'b-', linewidth=2)

ax.set_title("Live SLAM Trajectory")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_autoscale_on(True)

print("Plotting live trajectory...")

while True:
    try:
        data, _ = sock.recvfrom(1024)
        x, y, z, yaw = map(float, data.decode().split())

        xs.append(x)
        ys.append(y)

        line.set_data(xs, ys)

        ax.relim()
        ax.autoscale_view()

        plt.draw()
        plt.pause(0.001)

    except BlockingIOError:
        plt.pause(0.01)
