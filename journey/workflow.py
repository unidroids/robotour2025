import threading
import time
from datetime import datetime

from services import send_command
from util import log_event, parse_lidar_distance

PORT_CAMERA = 9001
PORT_LIDAR = 9002
PORT_DRIVE = 9003

demo_running = threading.Event()
stop_requested = threading.Event()

def journey_workflow(client_conn):
    log_event("WORKFLOW started.")
    demo_running.set()
    stop_requested.clear()
    try:
        # CAMERA sekvence
        client_conn.sendall(b"CAMERA, PING\n")
        send_command(PORT_CAMERA, "PING")
        send_command(PORT_CAMERA, "START")
        send_command(PORT_CAMERA, "QR")
        send_command(PORT_CAMERA, "STOP")
        # LIDAR sekvence
        send_command(PORT_LIDAR, "PING")
        send_command(PORT_LIDAR, "START")
        # Nekonečná smyčka - LIDAR, DISTANCE == -1
        while not stop_requested.is_set():
            resp = send_command(PORT_LIDAR, "DISTANCE")
            idx, dist = parse_lidar_distance(resp)
            client_conn.sendall(f"LIDAR DISTANCE: {resp}\n".encode())
            if idx != -1 and dist is not None:
                break
            time.sleep(0.2)
        # DRIVE sekvence
        send_command(PORT_DRIVE, "PING")
        send_command(PORT_DRIVE, "START")
        # Hlavní bezpečný cyklus s timeoutem
        timeout = 60
        time_end = time.time() + timeout
        last_state = None
        while not stop_requested.is_set() and (time.time() < time_end):
            resp = send_command(PORT_LIDAR, "DISTANCE")
            idx, dist = parse_lidar_distance(resp)
            if dist is not None and dist > 50:
                temp_state = "run"
            else:
                temp_state = "break"
            if temp_state != last_state:
                if temp_state == "break":
                    send_command(PORT_DRIVE, "BREAK")
                else:
                    send_command(PORT_DRIVE, "PWM 20 20")
                last_state = temp_state
            time.sleep(0.2)
        send_command(PORT_DRIVE, "STOP")
        send_command(PORT_LIDAR, "STOP")
        client_conn.sendall(b"WORKFLOW finished.\n")
        log_event("WORKFLOW finished.")
    except Exception as e:
        log_event(f"WORKFLOW ERROR: {e}")
    finally:
        demo_running.clear()

def stop_workflow():
    log_event("STOP requested.")
    stop_requested.set()
    send_command(PORT_CAMERA, "STOP", expect_response=False)
    send_command(PORT_DRIVE, "STOP", expect_response=False)
    send_command(PORT_LIDAR, "STOP", expect_response=False)
