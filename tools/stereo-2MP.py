#!/usr/bin/env python3
"""
Capture a pair of **lower‑resolution** stereo images (≈2 MP, 1640 × 1232) on Jetson
Orin Nano and rotate them so that the top of each saved frame points **forward**:

* **Left camera (sensor‑id 0)** → rotate **clockwise** (90 ° CW)
* **Right camera (sensor‑id 1)** → rotate **counter‑clockwise** (90 ° CCW)

Baseline between cameras ≈ 43 cm (mounted on chassis). The script warms up the
sensors for ~1 s (30 frames) to stabilise AE/AWB, then stores a single JPEG from
each camera in `/robot/data/logs/camera/`.

Usage (inside venv‑robotour):
```bash
python /robot/opt/tools/capture_stereo_rotated.py
```

Optional CLI flags allow changing resolution/fps.
"""
import cv2
import os
from datetime import datetime
import argparse

SAVE_DIR = "/robot/data/logs/camera"
DEFAULT_W, DEFAULT_H, DEFAULT_FPS = 1640, 1232, 30  # Jetson IMX219 sensor‑mode 1 ≈ 2 MP


def gst_pipeline(sensor_id: int, w: int, h: int, fps: int) -> str:
    """Return a GStreamer pipeline string for OpenCV."""
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={w}, height={h}, framerate={fps}/1 ! "
        "nvvidconv ! video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink drop=true"
    )


def capture_one(sensor_id: int, rotate_code: int, args) -> str:
    """Capture a single frame, rotate it, save JPEG, return filepath."""
    cap = cv2.VideoCapture(gst_pipeline(sensor_id, args.width, args.height, args.fps),
                           cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError(f"Camera {sensor_id} could not be opened")

    # Warm‑up to let AE/AWB settle
    for _ in range(args.warmup):
        cap.read()

    ret, frame = cap.read()
    cap.release()
    if not ret:
        raise RuntimeError(f"Failed to grab frame from camera {sensor_id}")

    frame = cv2.rotate(frame, rotate_code)
    os.makedirs(SAVE_DIR, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    side = "left" if sensor_id == 0 else "right"
    fname = os.path.join(SAVE_DIR, f"{side}_{ts}.jpg")
    cv2.imwrite(fname, frame, [int(cv2.IMWRITE_JPEG_QUALITY), args.quality])
    return fname


def main():
    parser = argparse.ArgumentParser(description="Capture rotated stereo JPEGs.")
    parser.add_argument("--width", type=int, default=DEFAULT_W, help="Output width px")
    parser.add_argument("--height", type=int, default=DEFAULT_H, help="Output height px")
    parser.add_argument("--fps", type=int, default=DEFAULT_FPS, help="Frame‑rate during warm‑up")
    parser.add_argument("--warmup", type=int, default=30, help="Number of warm‑up frames")
    parser.add_argument("--quality", type=int, default=92, help="JPEG quality (0‑100)")
    args = parser.parse_args()

    left_file = capture_one(0, cv2.ROTATE_90_CLOCKWISE, args)
    right_file = capture_one(1, cv2.ROTATE_90_COUNTERCLOCKWISE, args)

    print("Saved: ", left_file)
    print("Saved: ", right_file)


if __name__ == "__main__":
    main()
