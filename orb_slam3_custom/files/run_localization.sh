#!/bin/bash

ORB_DIR="/home/moussakathem/ORB_SLAM3"
VOC="$ORB_DIR/Vocabulary/ORBvoc.txt"
YAML="$ORB_DIR/Examples_old/Stereo/Stereo-Airsim-LOC.yaml"
MAP="$ORB_DIR/airsim_map.osa"

DATASET="/home/moussakathem/Tdataset"
CAM0_TS="$DATASET/mav0/cam0/times_cam0.txt"
CAM1_TS="$DATASET/mav0/cam1/times_cam1.txt"

OUT="CameraTrajectory_LOC.txt"

cd "$ORB_DIR"

taskset -c 2 ./Examples_old/Stereo/stereo_euroc_old \
    "$VOC" \
    "$YAML" \
    "$DATASET" "$CAM0_TS" \
    "$DATASET" "$CAM1_TS" \
    "$OUT"

