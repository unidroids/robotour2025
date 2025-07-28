import socket
import threading
import time
import cv2
import numpy as np
from pyzbar import pyzbar
import os
from datetime import datetime

HOST = '127.0.0.1'
PORT = 9001

latest_left = None
latest_right = None
frame_event_log = threading.Event()
frame_event_qr = threading.Event()

qr_result = None
qr_ready = threading.Event()
qr_lock = threading.Lock()
qr_running = False
qr_thread = None

shutdown_flag = False
client_threads = []
client_threads_lock = threading.Lock()

camera_running = False        # už běží hlavní smyčka kamer?
log_running    = False        # už běží logovací vlákno?
state_lock     = threading.Lock()

def crop_image(img):
    return img[150:-160] if img is not None else None

def save_stereo_image(left, right):
    now = datetime.now().strftime("%Y%m%d_%H%M%S")
    combined = np.hstack((left, right))
    filename = f"stereo_{now}.jpg"
    path = os.path.join("/data/logs/camera", filename)
    cv2.imwrite(path, combined)
    print(f"📂 Uloženo: {filename}")

def camera_loop():
    global latest_left, latest_right

    capL = cv2.VideoCapture("nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=1640, height=1232, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink", cv2.CAP_GSTREAMER)
    capR = cv2.VideoCapture("nvarguscamerasrc sensor-id=1 ! video/x-raw(memory:NVMM), width=1640, height=1232, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink", cv2.CAP_GSTREAMER)

    print("📷 Smyčka kamer spuštěna (2 MP GStreamer)")
    try:
        while not shutdown_flag:
            retL, frameL = capL.read()
            retR, frameR = capR.read()
            if not retL or not retR:
                continue
            latest_left = crop_image(frameL)
            latest_right = crop_image(frameR)
            frame_event_log.set()
            frame_event_qr.set()
            time.sleep(1)
    finally:
        capL.release()
        capR.release()
        print("🔛 Smyčka kamer ukončena")

def logging_loop():
    while not shutdown_flag:
        if frame_event_log.wait(timeout=60):
            frame_event_log.clear()
            if latest_left is not None and latest_right is not None:
                save_stereo_image(latest_left, latest_right)

def qr_worker():
    global qr_result, qr_running
    deadline = time.time() + 120

    while time.time() < deadline:
        if frame_event_qr.wait(timeout=1.0):
            if latest_right is None:
                continue
            image = crop_image(latest_right)
            codes = pyzbar.decode(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
            if codes:
                qr_result = codes[0].data.decode("utf-8")
                break

    qr_ready.set()
    with qr_lock:
        qr_running = False

def read_line(conn):
    """Blokuje, dokud nepřečte celou řádku ukončenou \\n.
       Vrací None, pokud klient zavřel socket."""
    buffer = b""
    while True:
        try:
            chunk = conn.recv(1)           # blokuje max 2 s díky settimeout
            if not chunk:                  # klient zavřel spojení
                return None
            if chunk == b"\n":             # konec příkazu
                break
            buffer += chunk
        except socket.timeout:
            if shutdown_flag:              # server se vypíná
                return None                # ukončit vlákno
            continue                       # jinak čekáme dál
    return buffer.decode().strip().upper()


def handle_client(conn, addr):
    global shutdown_flag, qr_running, qr_thread
    global state_lock, camera_running, log_running
    global qr_lock
    print(f"📱 Klient připojen: {addr}")
    conn.settimeout(2.0)

    with client_threads_lock:
        client_threads.append(threading.current_thread())

    try:
        while not shutdown_flag:
            cmd = read_line(conn)
            if not cmd:
                break
            print(f"📥 Příkaz: '{cmd}'")

            if cmd == "PING":
                conn.sendall(b"PONG\n")

            elif cmd == "RUN":
                with state_lock:
                    # kamera
                    if not camera_running:
                        threading.Thread(target=camera_loop, daemon=True).start()
                        camera_running = True
                        conn.sendall(b"CAM OK\n")
                    else:
                        conn.sendall(b"CAM ALREADY\n")

                    # logování
                    if not log_running:
                        threading.Thread(target=logging_loop, daemon=True).start()
                        log_running = True
                        conn.sendall(b"LOG OK\n")
                    else:
                        conn.sendall(b"LOG ALREADY\n")

            elif cmd == "STOP":
                with state_lock:
                    if camera_running:
                        camera_running = False        # v camera_loop kontroluj tuto proměnnou
                        conn.sendall(b"CAM STOPPED\n")
                    else:
                        conn.sendall(b"CAM NOTRUN\n")

                    if log_running:
                        log_running = False           # v logging_loop kontroluj tuto proměnnou
                        frame_event_log.set()         # probuď, aby vlákno mohlo skončit
                        conn.sendall(b"LOG STOPPED\n")
                    else:
                        conn.sendall(b"LOG NOTRUN\n")

            elif cmd == "QR":
                conn.sendall(b"QR STARTED\n")
                with qr_lock:
                    if not qr_running:
                        qr_result = None
                        qr_ready.clear()
                        qr_running = True
                        qr_thread = threading.Thread(target=qr_worker, daemon=True)
                        qr_thread.start()

                if qr_ready.wait(timeout=120):
                    if qr_result:
                        conn.sendall(f"QR FOUND: {qr_result}\n".encode())
                    else:
                        conn.sendall(b"QR TIMEOUT\n")
                else:
                    conn.sendall(b"QR TIMEOUT\n")
                try:
                    conn.shutdown(socket.SHUT_RDWR)
                except:
                    pass
                conn.close()
                return

            elif cmd == "EXIT":
                conn.sendall(b"BYE\n")
                break

            else:
                conn.sendall(b"ERR\n")

    except Exception as e:
        print(f"❌ Chyba: {e}")
    finally:
        try:
            conn.shutdown(socket.SHUT_RDWR)
        except:
            pass
        conn.close()
        with client_threads_lock:
            if threading.current_thread() in client_threads:
                client_threads.remove(threading.current_thread())
        print(f"🔌 Odpojeno: {addr}")

def start_server():
    global shutdown_flag
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(5)
    print(f"📷 robot-cameras server naslouchá na {HOST}:{PORT}")

    try:
        while not shutdown_flag:
            try:
                conn, addr = server.accept()
                t = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
                t.start()
            except socket.timeout:
                continue
    except KeyboardInterrupt:
        print("🛟 Ctrl+C – ukončuji server")
    finally:
        shutdown_flag = True
        print("⌛ Čekám na dokončení klientských vláken…")
        with client_threads_lock:
            for t in client_threads:
                t.join()
        print("✅ Všechna klientská vlákna ukončena")
        server.close()
        print("🔝 Port uvolněn, server ukončen")

if __name__ == "__main__":
    start_server()
