import cv2
import time
import os
from datetime import datetime

# Výstupní složka
SAVE_DIR = "/robot/data/logs/camera"
os.makedirs(SAVE_DIR, exist_ok=True)

# Kamera: GStreamer pipeline (pro sensor-id=0)
GST_PIPE = (
    "nvarguscamerasrc sensor-id=0 ! "
    "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=1/1 ! "
    "nvvidconv ! video/x-raw, format=BGRx ! "
    "videoconvert ! video/x-raw, format=BGR ! appsink"
)

# Inicializace kamery
cap = cv2.VideoCapture(GST_PIPE, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    raise RuntimeError("Kamera se nepodařila otevřít.")

print("Kamera aktivní, ukládám 10 snímků každou 1 sekundu...")

for i in range(30):
    ret, frame = cap.read()
    if not ret:
        print(f"Snímek {i+1} se nepodařilo získat.")
        continue

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(SAVE_DIR, f"left_{timestamp}.jpg")
    cv2.imwrite(filename, frame)
    print(f"[{i+1}/30] Uloženo: {filename}")

    time.sleep(1)

cap.release()
print("Hotovo.")
