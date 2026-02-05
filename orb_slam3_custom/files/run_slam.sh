# run_slam.sh
#!/bin/bash
set -euo pipefail
i=${1:? "pass run index"}
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1

out_cam="CameraTrajectory_run${i}.txt"
out_kf="KeyFrameTrajectory_run${i}.txt"

taskset -c 2 ./Examples_old/Stereo/stereo_euroc_old \
  Vocabulary/ORBvoc.txt \
  Examples_old/Stereo/Stereo-Airsim-640x480.yaml \
  ~/Tdataset ~/Tdataset/mav0/cam0/times_cam0.txt \
  ~/Tdataset ~/Tdataset/mav0/cam1/times_cam1.txt \
  "${out_cam}"


# ORB-SLAM3 writes KeyFrameTrajectory.txt in CWD; rename if present.
[ -f KeyFrameTrajectory.txt ] && mv KeyFrameTrajectory.txt "${out_kf}"
echo "Saved ${out_cam}  ${out_kf:-'(no KF file)'}"

