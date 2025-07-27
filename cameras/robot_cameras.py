import socket
import threading

import asyncio
import cv2

import os
from datetime import datetime
import time

# zakladní vlákna : ctení kamer, logování
running_loop = False
loop_thread = None

log_thread = None
log_running = False


# Sdílené snímky z kamer
latest_left = None
latest_right = None

# Synchronizace mezi smyčkami
frame_lock = threading.Lock()
frame_event = threading.Event()


HOST = '127.0.0.1'   # Lokální přístup
PORT = 9001          # Port pro pravou kameru (můžeme později rozšířit)

def read_line(conn):
    buffer = b""
    while not buffer.endswith(b"\n"):
        chunk = conn.recv(1)
        if not chunk:
            break
        buffer += chunk
    return buffer.decode("utf-8").strip().upper()

def handle_client(conn, addr):
    print(f"📡 Klient připojen: {addr}")
    global running_loop, loop_thread
    global log_running, log_thread
    try:
        with conn:
            while True:
                cmd = read_line(conn)
                if not cmd:
                    break
                print(f"📥 Příkaz: '{cmd}'")
                if cmd == "PING": # PING - communication test
                    conn.sendall(b"PONG\n")

                elif (cmd == "RUN" or cmd == "START"): # RUN - start internal loop
                    if not running_loop:
                        running_loop = True
                        loop_thread = threading.Thread(target=camera_loop_thread)
                        loop_thread.start()
                        conn.sendall(b"LOOP OK\n")
                    else:
                        conn.sendall(b"LOOP ALREADY\n")

                    if not log_running:
                        log_running = True
                        log_thread = threading.Thread(target=log_loop_thread)
                        log_thread.start()
                        conn.sendall(b"LOG OK\n")
                    else:
                        conn.sendall(b"LOG ALREADY\n")

                elif cmd == "STOP": # STOP - stops internal loop
                    if running_loop:
                        running_loop = False
                        frame_event.set()  # probudí vlákno, aby se ukončilo
                        conn.sendall(b"OK\n")
                    else:
                        conn.sendall(b"NOTRUN\n")

                elif cmd == "EXIT": # ukončí while smyčku a spojení
                    conn.sendall(b"BYE\n")
                    break  

                elif cmd == "LCAM":
                    conn.sendall(b"OK\n")
                elif cmd == "RCAM":
                    conn.sendall(b"OK\n")
                elif cmd == "QR":
                    conn.sendall(b"OK\n")
                else:
                    conn.sendall(b"ERR\n")
    except Exception as e:
        print(f"❌ Chyba: {e}")
    finally:
        print(f"🔌 Odpojeno: {addr}")

def start_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT))
    server.listen()
    print(f"📷 robot-cameras server naslouchá na {HOST}:{PORT}")

    while True:
        conn, addr = server.accept()
        thread = threading.Thread(target=handle_client, args=(conn, addr))
        thread.start()


def gst_pipeline(sensor_id: int, w: int = 1640, h: int = 1232, fps: int = 30) -> str:
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={w}, height={h}, framerate={fps}/1 ! "
        "nvvidconv ! video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink drop=true"
    )

def log_loop_thread():
    global log_running
    print("📝 Logovací vlákno spuštěno")
    path = "/robot/data/logs/camera"
    os.makedirs(path, exist_ok=True)

    while log_running:
        frame_event.wait(timeout=5.0)  # počká na nový snímek (nebo každých 5s)

        with frame_lock:
            left = latest_left.copy() if latest_left is not None else None
            right = latest_right.copy() if latest_right is not None else None
            frame_event.clear()

        if left is not None and right is not None:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            combined = cv2.hconcat([left, right])
            cv2.imwrite(f"{path}/stereo_{ts}.jpg", combined, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            print(f"💾 Uloženo: stereo_{ts}.jpg")

        time.sleep(3)  # zapisuj foto každých x sekund

    print("🛑 Logovací vlákno ukončeno")


def camera_loop_thread():
    global running_loop, latest_left, latest_right
    print("📷 Smyčka kamer spuštěna (2 MP GStreamer)")
    try:
        capL = cv2.VideoCapture(gst_pipeline(0), cv2.CAP_GSTREAMER)
        capR = cv2.VideoCapture(gst_pipeline(1), cv2.CAP_GSTREAMER)

        if not capL.isOpened() or not capR.isOpened():
            print("❌ Nelze otevřít kamery")
            running_loop = False
            return

        while running_loop:
            retL, frameL = capL.read()
            retR, frameR = capR.read()

        if retL:
            frameL = cv2.rotate(frameL, cv2.ROTATE_90_CLOCKWISE)
            frameL = frameL[150:-165, :]  # ořízni horních 150 a spodních 160 px
        if retR:
            frameR = cv2.rotate(frameR, cv2.ROTATE_90_COUNTERCLOCKWISE)
            frameR = frameR[150:-165, :]


            with frame_lock:
                if retL:
                    latest_left = frameL.copy()
                if retR:
                    latest_right = frameR.copy()
                if retL and retR:
                    frame_event.set()  # signalizujeme že snímky jsou připraveny

            time.sleep(1.0)  # můžeš zkrátit např. na 0.2 pro plynulejší běh

    except Exception as e:
        print(f"❌ Kamera loop chyba: {e}")
    finally:
        capL.release()
        capR.release()
        print("🛑 Smyčka kamer ukončena")

if __name__ == "__main__":
    start_server()
