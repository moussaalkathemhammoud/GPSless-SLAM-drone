import os, glob, shutil, math

# ----------- CONFIG -----------
WIN_BASE = "/mnt/d/AirSim/PythonClient/Tdataset/mav0"  # Windows path in WSL form
OUT_BASE = os.path.expanduser("~/Tdataset/mav0")       # Linux destination
FPS = 30.0                                             # must match recording
DT = 1.0 / FPS
# ------------------------------

os.makedirs(os.path.join(OUT_BASE, "cam0", "data"), exist_ok=True)
os.makedirs(os.path.join(OUT_BASE, "cam1", "data"), exist_ok=True)
os.makedirs(os.path.join(OUT_BASE, "imu0"), exist_ok=True)

left_imgs  = sorted(glob.glob(os.path.join(WIN_BASE, "cam0", "data", "*.png")))
right_imgs = sorted(glob.glob(os.path.join(WIN_BASE, "cam1", "data", "*.png")))
N = min(len(left_imgs), len(right_imgs))
print(f"Found {N} stereo pairs")

t0_path = os.path.join(OUT_BASE, "cam0", "times_cam0.txt")
t1_path = os.path.join(OUT_BASE, "cam1", "times_cam1.txt")

with open(t0_path, "w") as t0, open(t1_path, "w") as t1:
    for i, (l, r) in enumerate(zip(left_imgs, right_imgs)):
        t = i * DT
        timestamp = f"{t:.6f}"
        new_name = f"{timestamp}.png"

        shutil.copy2(l, os.path.join(OUT_BASE, "cam0", "data", new_name))
        shutil.copy2(r, os.path.join(OUT_BASE, "cam1", "data", new_name))

        t0.write(f"{timestamp}\n")
        t1.write(f"{timestamp}\n")

# Copy IMU data (if present)
imu_src = os.path.join(WIN_BASE, "imu0", "data.csv")
imu_dst = os.path.join(OUT_BASE, "imu0", "data.csv")
if os.path.exists(imu_src):
    shutil.copy2(imu_src, imu_dst)
    print("Copied IMU data.")

print("✅ Conversion complete.")
print("Output ready at:", OUT_BASE)
