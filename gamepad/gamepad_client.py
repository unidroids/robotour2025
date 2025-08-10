#!/usr/bin/env python3
# Per-connection handler: PING | START | DATA | STOP
import json, socket

from gamepad_core import init_gamepad, start_compute_once, stop_all
from gamepad_core import cond, stop_event, latest_payload
from datalogger   import start_dataloger_once
from gamepad_control import register_client, unregister_client, request_shutdown, close_all_clients, shutdown_event

def _send_line(conn, text):
    try:
        conn.sendall((text + "\n").encode("utf-8"))
    except Exception as e:
        print(f"[CLIENT] send_line error: {e}")

# def _send_json(conn, obj):
#     try:
#         conn.sendall(json.dumps(obj, ensure_ascii=False).encode("utf-8"))  # bez newline
#     except Exception as e:
#         print(f"[CLIENT] send_json error: {e}")

def handle_client(conn: socket.socket, addr):
    global latest_payload, cond, stop_event
    print(f"[SERVER] Klient připojen: {addr}")
    register_client(conn)
    try:
        buf = b""
        with conn:
            while not shutdown_event.is_set():
                data = conn.recv(1024)
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    cmd = line.decode("utf-8", errors="replace").strip().upper()

                    if cmd == "PING":
                        _send_line(conn, "PONG")

                    elif cmd == "START":
                        if init_gamepad():
                            g = start_compute_once()       # běží jen jednou
                            l = start_dataloger_once()     # běží jen jednou
                            msg = "OK STARTED"
                        else:
                            msg = "GAMEPAD NOT FOUND"
                        _send_line(conn, msg)

                    elif cmd == "DATA":
                        with cond:
                            if latest_payload == None:
                                _send_line(conn, "NO DATA")      # empty \n
                            else:
                                cond.wait_for(lambda: stop_event.is_set())
                                payload = latest_payload
                                _send_line(conn, payload)      # JSON s \n

                    elif cmd == "STOP":
                        _send_line(conn, "OK STOPPING GAMEPAD")
                        stop_all()                         # jen gamepad+datalogger; server běží dál
                        _send_line(conn, "OK STOPPED")

                    else:
                        _send_line(conn, f"ERR Unknown command: {cmd}")

    except ConnectionResetError:
        print(f"[SERVER] {addr} reset connection")
    except Exception as e:
        print(f"[SERVER] Chyba klienta {addr}: {e}")
    finally:
        try:
            conn.close()
        except Exception:
            pass
        unregister_client(conn)
        print(f"[SERVER] Klient odpojen: {addr}")
