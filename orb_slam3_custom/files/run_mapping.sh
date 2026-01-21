#!/bin/bash
set -euo pipefail

export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1

taskset -c 2 ./Examples_old/Stereo/stereo_euroc_old \
  Vocabulary/ORBvoc.txt \
  Examples_old/Stereo/Stereo-Airsim-MAP.yaml \
  ~/Tdataset ~/Tdataset/mav0/cam0/times_cam0.txt \
  ~/Tdataset ~/Tdataset/mav0/cam1/times_cam1.txt \
  "CameraTrajectory_MAP.txt"
