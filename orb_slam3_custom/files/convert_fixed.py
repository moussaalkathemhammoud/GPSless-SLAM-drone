import os, glob, shutil

RAW = "/mnt/d/AirSim/PythonClient/Tdataset3/mav0"
OUT = os.path.expanduser("~/Tdataset/mav0")

FPS = 30
DT = 1.0 / FPS

os.makedirs(f"{OUT}/cam0/data", exist_ok=True)
os.makedirs(f"{OUT}/cam1/data", exist_ok=True)
os.makedirs(f"{OUT}/imu0", exist_ok=True)

left = sorted(glob.glob(f"{RAW}/cam0/data/*.png"))
right = sorted(glob.glob(f"{RAW}/cam1/data/*.png"))

N = min(len(left), len(right))
print("Found", N, "pairs")

with open(f"{OUT}/cam0/times_cam0.txt", "w") as t0, \
     open(f"{OUT}/cam1/times_cam1.txt", "w") as t1:
    
    for i in range(N):
        ts = i * DT
        ts_str = f"{ts:.6f}"

        shutil.copy2(left[i],  f"{OUT}/cam0/data/{ts_str}.png")
        shutil.copy2(right[i], f"{OUT}/cam1/data/{ts_str}.png")

        t0.write(ts_str + "\n")
        t1.write(ts_str + "\n")

print("DONE")

