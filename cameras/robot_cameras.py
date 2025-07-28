import socket
import threading

import asyncio
import cv2

import os
from datetime import datetime
import time

from pyzbar import pyzbar
import traceback

# vlákna klintů
shutdown_flag = False

# zakladní vlákna : ctení kamer, logování
state_lock = threading.Lock()
loop_running = False
log_running = False
loop_thread = None
log_thread = None


# Sdílené snímky z kamer
latest_left = None
latest_right = None

# Synchronizace mezi smyčkami
frame_lock = threading.Lock()
frame_event_log = threading.Event()
frame_event_qr = threading.Event()

#promenne ke QR kodu
qr_running = False
qr_thread = None
qr_lock = threading.Lock()
qr_result = None
qr_ready = threading.Event()


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
    global loop_running, loop_thread
    global log_running, log_thread
    global shutdown_flag
    global qr_running, qr_thread, qr_lock, qr_result, qr_ready
    try:
        conn.settimeout(2.0)
        with conn:
            while True:
                try:
                    cmd = read_line(conn)
                    if not cmd:
                        break
                except socket.timeout:
                    if (shutdown_flag):
                        conn.sendall(b"SERVER SHUTDOWN\n")
                        conn.sendall(b'')
                        conn.shutdown(socket.SHUT_RDWR)
                        conn.close()
                        break
                    else:
                        continue  # jinak jen čekáme dál


                print(f"📥 Příkaz: '{cmd}'")

                if cmd == "PING": # PING - communication test
                    conn.sendall(b"PONG\n")

                elif cmd == "HI": # PING - communication test
                    conn.sendall(b"HI\n")

                elif (cmd == "RUN" or cmd == "START"): # RUN - start internal loop
                    with state_lock:
                        if not loop_running:
                            loop_running = True
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
                    with state_lock:
                        if loop_running:
                            loop_running = False
                            conn.sendall(b"LOOP OK\n")
                        else:
                            conn.sendall(b"LOOP NOTRUN\n")

                        if log_running:
                            log_running = False
                            frame_event_log.set()  # probudí vlákno, aby se ukončilo
                            frame_event_qr.set()  # probudí vlákno, aby se ukončilo
                            conn.sendall(b"LOG OK\n")
                        else:
                            conn.sendall(b"LOG NOTRUN\n")

                elif cmd == "EXIT": # ukončí while smyčku a spojení
                    conn.sendall(b"BYE\n")
                    conn.sendall(b'')
                    conn.shutdown(socket.SHUT_RDWR)
                    conn.close()
                    break  

                elif cmd == "SHUTDOWN": # ukončí while smyčku a spojení
                    shutdown_flag = True

                elif cmd == "QR":

                    print(f"🧾 QR STARTED")

                    with qr_lock:
                        if not qr_running:
                            qr_running = True
                            qr_thread = threading.Thread(target=qr_worker, daemon=True)
                            qr_thread.start()

                    # počkej na výsledek nebo timeout
                    deadline = time.time() + 120
                    while (time.time() < deadline and not shutdown_flag ):                    
                        if qr_ready.wait(timeout=2):
                            if qr_result:
                                conn.sendall(f"QR:{qr_result}\n".encode())
                                print(f"🧾 QR FOUND:{qr_result}\n")
                                conn.shutdown(socket.SHUT_RDWR)
                                conn.close()
                                break
                        
                    if (time.time() < deadline and qr_result is None):
                        conn.sendall("QR:NONE\n".encode())
                        print(f"🧾 QR TIMEOUT\n")
                        conn.shutdown(socket.SHUT_RDWR)
                        conn.close()
                        break               

                elif cmd == "LCAM":
                    conn.sendall(b"OK\n")
                elif cmd == "RCAM":
                    conn.sendall(b"OK\n")
                else:
                    conn.sendall(b"ERR\n")
    except Exception as e:
        print(f"❌ Chyba: {e}\n📍 Stack:\n{traceback.format_exc()}")
    finally:
        print(f"🔌 Odpojeno: {addr}")

def qr_worker():
    global qr_result, shutdown_flag, qr_ready, frame_event_qr, latest_right, qr_lock, qr_running

    deadline = time.time() + 120
    qr_result = None
    frame_event_qr.clear()

    while (time.time() < deadline and not shutdown_flag ):
        if frame_event_qr.wait(timeout=10):
            frame_event_qr.clear()
            if latest_right is None:
                continue

            codes = pyzbar.decode(cv2.cvtColor(latest_right, cv2.COLOR_BGR2GRAY))
            print(f"🧾 QR data ... {len(codes)}")

            for code in codes:
                data = code.data.decode("utf-8")
                if data.startswith("geo:"):  # nebo jiný filtr
                    qr_result = data
                    break

        if qr_result:
            qr_ready.set()
            break

    with qr_lock:
        qr_running = False


def start_server():
    global loop_running, log_running
    global shutdown_flag

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT))
    server.listen()
    print(f"📷 robot-cameras server naslouchá na {HOST}:{PORT}")

    try:
        while not shutdown_flag:
            server.settimeout(2.0)  # umožní kontrolu shutdown_flag
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            thread = threading.Thread(target=handle_client, args=(conn, addr))
            thread.start()
    except KeyboardInterrupt:
        shutdown_flag = True
        print("\n🧯 Ctrl+C – ukončuji server")
    finally:
        server.close()
        log_running=False
        loop_running=False
        frame_event_qr.set()
        frame_event_log.set()  
        time.sleep(0.1)
        print("🛑 Port uvolněn, server ukončen")


def gst_pipeline(sensor_id: int, w: int = 1640, h: int = 1232, fps: int = 10) -> str:
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
        frame_event_log.wait(timeout=5.0)  # počká na nový snímek (nebo každých 5s)

        with frame_lock:
            left = latest_left.copy() if latest_left is not None else None
            right = latest_right.copy() if latest_right is not None else None
            frame_event_log.clear()

        if left is not None and right is not None:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            combined = cv2.hconcat([left, right])
            cv2.imwrite(f"{path}/stereo_{ts}.jpg", combined, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            print(f"💾 Uloženo: stereo_{ts}.jpg")

        time.sleep(3)  # zapisuj foto každých x sekund

    print("🛑 Logovací vlákno ukončeno")


def camera_loop_thread():
    global loop_running, latest_left, latest_right
    print("📷 Smyčka kamer spuštěna (2 MP GStreamer)")
    try:
        capL = cv2.VideoCapture(gst_pipeline(0), cv2.CAP_GSTREAMER)
        capR = cv2.VideoCapture(gst_pipeline(1), cv2.CAP_GSTREAMER)

        if not capL.isOpened() or not capR.isOpened():
            print("❌ Nelze otevřít kamery")
            loop_running = False
            return

        while loop_running:
            t0 = time.time()
            retL, frameL = capL.read()
            t1 = time.time()
            retR, frameR = capR.read()
            t2 = time.time()

            dt_left = (t1 - t0) * 1000  # ms
            dt_right = (t2 - t1) * 1000  # ms
            dt_total = (t2 - t0) * 1000

            print(f"⏱ Kamera L: {dt_left:.1f} ms, R: {dt_right:.1f} ms, Δ celkem: {dt_total:.1f} ms")

            if retL and retR:
                frameL = cv2.rotate(frameL, cv2.ROTATE_90_CLOCKWISE)
                frameL = frameL[150:-165, :]
                frameR = cv2.rotate(frameR, cv2.ROTATE_90_COUNTERCLOCKWISE)
                frameR = frameR[150:-165, :]

                with frame_lock:
                    latest_left = frameL.copy()
                    latest_right = frameR.copy()
                    frame_event_log.set()
                    frame_event_qr.set()

            time.sleep(1.0) #pauza mezi snímky


    except Exception as e:
        print(f"❌ Kamera loop chyba: {e}")
    finally:
        capL.release()
        capR.release()
        print("🛑 Smyčka kamer ukončena")

if __name__ == "__main__":
    start_server()
