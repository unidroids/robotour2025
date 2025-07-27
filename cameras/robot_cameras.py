import socket
import threading

import asyncio
import cv2

import os
from datetime import datetime
import time

running_loop = False
loop_thread = None

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
                        conn.sendall(b"OK\n")
                    else:
                        conn.sendall(b"ALREADY\n")

                elif cmd == "STOP": # STOP - stops internal loop
                    if running_loop:
                        running_loop = False
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

def camera_loop_thread():
    global running_loop
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
            if retR:
                frameR = cv2.rotate(frameR, cv2.ROTATE_90_COUNTERCLOCKWISE)

            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = "/robot/data/logs/camera"
            os.makedirs(path, exist_ok=True)

            if retL:
                cv2.imwrite(f"{path}/left_{ts}.jpg", frameL, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
            if retR:
                cv2.imwrite(f"{path}/right_{ts}.jpg", frameR, [int(cv2.IMWRITE_JPEG_QUALITY), 92])

            time.sleep(1.0)  # každou sekundu

    except Exception as e:
        print(f"❌ Kamera loop chyba: {e}")
    finally:
        capL.release()
        capR.release()
        print("🛑 Smyčka kamer ukončena")


if __name__ == "__main__":
    start_server()
