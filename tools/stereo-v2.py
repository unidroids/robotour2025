#!/usr/bin/env python3
"""
Capture a *pair* of 8‑MP stereo images on Jetson Orin using two IMX219 cameras.

• Baseline: 43 cm (left = sensor‑id 0, right = sensor‑id 1)
• Resolution: 3280 × 2464 (sensor‑mode 0)
• Warm‑up: let each camera stream ~30 frames so AE/AWB can settle
• Saves JPEGs to /robot/data/logs/camera/ in the form
    left_<YYYYMMDD_HHMMSS>.jpg  /  right_<YYYYMMDD_HHMMSS>.jpg

Tested on JetPack 6 with OpenCV 4.
"""
import cv2
import os
from datetime import datetime
from pathlib import Path

# -------- configuration -----------------------------------------------------
SAVE_DIR = Path("/robot/data/logs/camera")
SAVE_DIR.mkdir(parents=True, exist_ok=True)

# mapping of logical names → CSI sensor‑ids
SENSORS = {
    "left": 0,
    "right": 1,
}

# number of frames to discard for AE/AWB warm‑up
WARMUP_FRAMES = 50

# GStreamer pipeline template (IMX219, 8 MP, 21 fps)
PIPE_FMT = (
    "nvarguscamerasrc sensor-id={sid} sensor-mode=0 ! "
    "video/x-raw(memory:NVMM), width=3280, height=2464, format=NV12, framerate=21/1 ! "
    "nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"
)

# ---------------------------------------------------------------------------
def capture_single(name: str, sid: int) -> Path:
    """Capture a single frame from `sid`, return saved path."""
    pipeline = PIPE_FMT.format(sid=sid)
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera sensor-id {sid} ({name})")

    # warm‑up AE/AWB
    for _ in range(WARMUP_FRAMES):
        cap.read()

    ret, frame = cap.read()
    cap.release()
    if not ret:
        raise RuntimeError(f"Failed to capture frame from {name} (sensor {sid})")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    fname = SAVE_DIR / f"{name}_{timestamp}.jpg"
    cv2.imwrite(str(fname), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    return fname


if __name__ == "__main__":
    print("Capturing stereo pair @ 8 MP …")
    saved = {}
    for name, sid in SENSORS.items():
        try:
            saved[name] = capture_single(name, sid)
            print(f"→ {name}: {saved[name]}")
        except Exception as e:
            print(f"ERROR on {name}: {e}")

    if len(saved) == 2:
        print("\nStereo pair captured successfully.")
    else:
        print("\n⚠️  Could not capture both images – see errors above.")
