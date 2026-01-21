import os, glob, shutil

RAW = "/mnt/d/AirSim/PythonClient/Tdataset2/mav0"  # pick your correct dataset
OUT = os.path.expanduser("~/Tdataset/mav0")

FPS = 30.0
DT = 1.0 / FPS

os.makedirs(f"{OUT}/cam0/data", exist_ok=True)
os.makedirs(f"{OUT}/cam1/data", exist_ok=True)
os.makedirs(f"{OUT}/imu0", exist_ok=True)

left = sorted(glob.glob(f"{RAW}/cam0/data/*.png"))
right = sorted(glob.glob(f"{RAW}/cam1/data/*.png"))

N = min(len(left), len(right))
print("Found", N, "pairs")

t0 = open(f"{OUT}/cam0/times_cam0.txt", "w")
t1 = open(f"{OUT}/cam1/times_cam1.txt", "w")

for i in range(N):
    ts = int(i * DT * 1e9)   # integer nanoseconds
    ts_name = f"{ts}.png"

    shutil.copy2(left[i], f"{OUT}/cam0/data/{ts_name}")
    shutil.copy2(right[i], f"{OUT}/cam1/data/{ts_name}")

    t0.write(f"{ts}\n")
    t1.write(f"{ts}\n")

t0.close()
t1.close()

imu_src = f"{RAW}/imu0/data.csv"
imu_dst = f"{OUT}/imu0/data.csv"
if os.path.exists(imu_src):
    shutil.copy2(imu_src, imu_dst)

print("DONE")
