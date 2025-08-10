#!/usr/bin/env python3
# Per-connection handler: PING | START | DATA | STOP
import json, socket

from gamepad_core import init_gamepad, start_compute_once, get_latest_payload, stop_all
from datalogger   import start_dataloger_once
from gamepad_control import register_client, unregister_client, request_shutdown, close_all_clients, shutdown_event

def _send_line(conn, text):
    try:
        conn.sendall((text + "\n").encode("utf-8"))
    except Exception as e:
        print(f"[CLIENT] send_line error: {e}")

def _send_json(conn, obj):
    try:
        conn.sendall(json.dumps(obj, ensure_ascii=False).encode("utf-8"))  # bez newline
    except Exception as e:
        print(f"[CLIENT] send_json error: {e}")

def handle_client(conn: socket.socket, addr):
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
                        init_gamepad()                 # idempotentní (pokud už běží, jen vypíše stav)
                        g = start_compute_once()       # běží jen jednou
                        l = start_dataloger_once()     # běží jen jednou
                        msg = "OK STARTED"
                        if g or l:
                            msg += " (new)"
                        _send_line(conn, msg)

                    elif cmd == "DATA":
                        payload = get_latest_payload()
                        _send_json(conn, payload)      # JSON bez \n

                    elif cmd == "STOP":
                        _send_line(conn, "OK STOPPING")
                        stop_all()                     # ukonči gamepad + signály
                        #request_shutdown()             # požádej server o zavření soketu
                        #close_all_clients()            # zavři všechny klienty (včetně sebe)
                        return

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
