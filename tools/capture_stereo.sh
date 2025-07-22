#!/bin/bash

DIR="/robot/data/logs/camera"
mkdir -p "$DIR"

STAMP=$(date +"%Y%m%d_%H%M%S")

echo "Zahřívání kamery pro automatickou expozici..."

# Krátké spuštění kamery bez uložení (cca 1 sekunda)
gst-launch-1.0 -e nvarguscamerasrc sensor-id=0 num-buffers=30 ! \
  'video/x-raw(memory:NVMM), width=1280, height=720, format=NV12' ! \
  fakesink

# Poté teprve uložit jeden snímek
gst-launch-1.0 -e nvarguscamerasrc sensor-id=0 num-buffers=1 ! \
  'video/x-raw(memory:NVMM), width=1280, height=720, format=NV12' ! \
  nvvidconv ! videoconvert ! jpegenc ! filesink location="$DIR/left_${STAMP}.jpg"

echo "Hotovo: $DIR/left_${STAMP}.jpg"
