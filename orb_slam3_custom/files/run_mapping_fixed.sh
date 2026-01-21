#!/bin/bash
set -euo pipefail
export OMP_NUM_THREADS=1

YAML="Examples_old/Stereo/Stereo-Airsim-640x480.yaml"

OUT_CAM="CameraTrajectory_MAP.txt"
OUT_KF="KeyFrameTrajectory_MAP.txt"
OUT_MAP="airsim_map.bin"

taskset -c 2 ./Examples_old/Stereo/stereo_euroc_old \
  Vocabulary/ORBvoc.txt \
  "$YAML" \
  ~/Tdataset/mav0/cam0 \
  ~/Tdataset/mav0/cam0/times_cam0.txt \
  ~/Tdataset/mav0/cam1 \
  ~/Tdataset/mav0/cam1/times_cam1.txt \
  "$OUT_CAM" \
  --save-map "$OUT_MAP"

[ -f KeyFrameTrajectory.txt ] && mv KeyFrameTrajectory.txt "$OUT_KF"

echo "Saved:"
echo "  Trajectory: $OUT_CAM"
echo "  Keyframes:  $OUT_KF"
echo "  Map:        $OUT_MAP"

