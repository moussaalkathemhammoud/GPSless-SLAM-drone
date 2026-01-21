import numpy as np
import matplotlib.pyplot as plt

# Load trajectory (TUM format)
data = np.loadtxt("airsim_trajectory_tum.txt")

# Columns: t x y z qx qy qz qw
x = data[:,1]
y = data[:,2]
z = data[:,3]

fig = plt.figure(figsize=(8,6))
ax = fig.add_subplot(111, projection='3d')

ax.plot(x, y, z, linewidth=1)
ax.scatter(x[0], y[0], z[0], c='green', s=50, label="Start")
ax.scatter(x[-1], y[-1], z[-1], c='red', s=50, label="End")

ax.set_title("ORB-SLAM3 AirSim Trajectory")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.legend()

plt.tight_layout()
plt.figure(figsize=(8,8))
plt.plot(x, y, linewidth=1)
plt.scatter(x[0], y[0], c='green', s=50, label="Start")
plt.scatter(x[-1], y[-1], c='red', s=50, label="End")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.title("Top-Down SLAM Trajectory (X–Y)")
plt.xlabel("X")
plt.ylabel("Y")
plt.savefig("trajectory_topdown.png", dpi=200)
print("Saved trajectory_topdown.png")

