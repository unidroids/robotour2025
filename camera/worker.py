# worker.py – pracovní vlákna a sdílený stav (Robotour 2025 cameras)
import cv2
import os
import time
import threading
from datetime import datetime
from collections import deque
from pyzbar import pyzbar
import numpy as np
from pathlib import Path

# -------------------------------
# Sdílený stav
# -------------------------------
shutdown_flag = threading.Event()

# Kamera
loop_running = False
loop_thread = None

# Logování
log_running = False
log_thread = None

# QR detekce
qr_running = False
qr_thread = None
qr_lock = threading.Lock()
qr_result = None
qr_ready = threading.Event()

# Kruhové buffery pro snímky
BUFFER_SIZE = 3
left_buf = deque(maxlen=BUFFER_SIZE)
right_buf = deque(maxlen=BUFFER_SIZE)
frame_seq = 0
frame_cond = threading.Condition()

# -------------------------------
# Fisheye korekce
# -------------------------------
calib_dir = Path(__file__).parent
calib_files = {
    "left": calib_dir / "camera_fisheye_calib_c0.npz",
    "right": calib_dir / "camera_fisheye_calib_c1.npz",
}
mapL1, mapL2, mapR1, mapR2 = None, None, None, None

def load_fisheye_maps(w: int, h: int):
    global mapL1, mapL2, mapR1, mapR2
    try:
        dL = np.load(calib_files["left"])
        dR = np.load(calib_files["right"])
        KL, DL = dL["K"], dL["D"]
        KR, DR = dR["K"], dR["D"]

        KLnew, KRnew = KL.copy(), KR.copy()

        mapL1, mapL2 = cv2.fisheye.initUndistortRectifyMap(
            KL, DL, np.eye(3), KLnew, (w, h), cv2.CV_16SC2
        )
        mapR1, mapR2 = cv2.fisheye.initUndistortRectifyMap(
            KR, DR, np.eye(3), KRnew, (w, h), cv2.CV_16SC2
        )
        print(f"✅ Fisheye mapy načteny ({w}x{h})")
    except Exception as e:
        print(f"⚠️ Nelze načíst fisheye kalibraci: {e}")
        mapL1 = mapL2 = mapR1 = mapR2 = None

# -------------------------------
# Utility – GStreamer pipeline
# -------------------------------
def gst_pipeline(sensor_id: int, f: int, w: int = 1000, h: int = 800, fps: int = 5) -> str:
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={w}, height={h}, framerate={fps}/1 ! "
        f"nvvidconv flip-method={f} ! video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink drop=true max-buffers=1 sync=false"
    )

# -------------------------------
# Kamera vlákno
# -------------------------------
def camera_loop_thread():
    global loop_running, frame_seq
    print("📷 Smyčka kamer spuštěna")

    width, height = 1000 - 200, 800
    capL = cv2.VideoCapture(gst_pipeline(0, 3), cv2.CAP_GSTREAMER)
    capR = cv2.VideoCapture(gst_pipeline(1, 1), cv2.CAP_GSTREAMER)

    if not capL.isOpened() or not capR.isOpened():
        print("❌ Nelze otevřít kamery")
        loop_running = False
        return

    load_fisheye_maps(width, height)

    try:
        while loop_running and not shutdown_flag.is_set():
            retL, frameL = capL.read()
            retR, frameR = capR.read()

            if retL and retR:
                if mapL1 is not None:
                    frameL = cv2.remap(frameL, mapL1, mapL2, interpolation=cv2.INTER_LINEAR)
                if mapR1 is not None:
                    frameR = cv2.remap(frameR, mapR1, mapR2, interpolation=cv2.INTER_LINEAR)

                with frame_cond:
                    left_buf.append(frameL)
                    right_buf.append(frameR)
                    frame_seq += 1
                    frame_cond.notify_all()

            time.sleep(0.05)  # ~20 FPS max
    except Exception as e:
        print(f"❌ Kamera loop chyba: {e}")
    finally:
        capL.release()
        capR.release()
        print("🛑 Smyčka kamer ukončena")

# -------------------------------
# Logovací vlákno
# -------------------------------
def log_loop_thread():
    global log_running, frame_seq
    print("📝 Logovací vlákno spuštěno")

    path = "/robot/data/logs/camera"
    os.makedirs(path, exist_ok=True)
    last_seq = 0

    while log_running and not shutdown_flag.is_set():
        with frame_cond:
            frame_cond.wait_for(lambda: frame_seq > last_seq or not log_running, timeout=2)
            if frame_seq == last_seq:
                continue
            last_seq = frame_seq
            left = left_buf[-1]
            right = right_buf[-1]

        if left is not None and right is not None:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            combined = cv2.hconcat([left, right])
            filename = f"{path}/stereo_{ts}.jpg"
            cv2.imwrite(filename, combined, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            print(f"💾 Uloženo: {filename}")

        time.sleep(3)

    print("🛑 Logovací vlákno ukončeno")

# -------------------------------
# QR worker
# -------------------------------
# worker.py
def qr_worker():
    global qr_running, qr_result, qr_lock
    print("🧾 QR worker spuštěn")
    deadline = time.time() + 120
    last_seq = 0

    qr_ready.clear()

    while time.time() < deadline and not shutdown_flag.is_set() and loop_running:
        with frame_cond:
            frame_cond.wait_for(lambda: frame_seq > last_seq or shutdown_flag.is_set(), timeout=2)
            if frame_seq == last_seq:
                continue
            last_seq = frame_seq
            latest = right_buf[-1]

        codes = pyzbar.decode(cv2.cvtColor(latest, cv2.COLOR_BGR2GRAY))
        print(f"🧾 QR data … {len(codes)} kandidátů")

        for code in codes:
            data = code.data.decode("utf-8")
            print(f"🧾 QR DATA: {data}")
            if data.startswith("geo:"):
                with qr_lock:             # <<< chrání zápis
                    qr_result = data
                qr_ready.set()
                print(f"🧾 QR FOUND: {qr_result}")
                break

        if qr_ready.is_set():
            break

    with qr_lock:
        qr_running = False
    print("🧾 QR worker ukončen")


# -------------------------------
# API pro client.py
# -------------------------------
def start_camera_loop() -> bool:
    global loop_running, loop_thread
    if loop_running:
        return False
    loop_running = True
    loop_thread = threading.Thread(target=camera_loop_thread, daemon=True)
    loop_thread.start()
    return True

def stop_camera_loop():
    global loop_running
    loop_running = False

def start_log_loop() -> bool:
    global log_running, log_thread
    if log_running:
        return False
    log_running = True
    log_thread = threading.Thread(target=log_loop_thread, daemon=True)
    log_thread.start()
    return True

def stop_log_loop():
    global log_running
    log_running = False

def start_qr_worker() -> bool:
    global qr_running, qr_thread, qr_result
    if not loop_running:
        return False
    with qr_lock:
        if qr_running:
            return True
        qr_running = True
        qr_result = None   # reset při startu
        qr_thread = threading.Thread(target=qr_worker, daemon=True)
        qr_thread.start()
        return True

def stop_all():
    stop_camera_loop()
    stop_log_loop()
    with qr_lock:
        if qr_running:
            # QR worker doběhne sám
            pass
