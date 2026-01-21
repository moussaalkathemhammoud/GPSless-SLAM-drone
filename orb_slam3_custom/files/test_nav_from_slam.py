import airsim
import time

# Connect
client = airsim.MultirotorClient(ip="172.20.144.1")
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Take off
client.takeoffAsync().join()
time.sleep(2)

# === TARGET FROM SLAM (converted) ===
x = 4.89   # forward
y = -11.19   # right
z = -17.42  # UP is negative in AirSim

print("Flying to SLAM target...")
client.moveToPositionAsync(x, y, z, velocity=1.5).join()

print("Reached target, hovering...")
client.hoverAsync().join()

