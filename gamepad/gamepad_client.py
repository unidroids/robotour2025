#!/usr/bin/env python3
# Per-connection handler: PING | START | DATA | STOP
import socket
import gamepad_core as core
from datalogger import start_dataloger_once
from gamepad_control import register_client, unregister_client, shutdown_event

def _send_line(conn, text: str):
    try:
        conn.sendall((text + "\n").encode("utf-8"))
    except Exception as e:
        print(f"[CLIENT] send_line error: {e}")

def handle_client(conn: socket.socket, addr):
    print(f"[SERVER] Klient připojen: {addr}")
    register_client(conn)
    try:
        buf = b""
        last_idx = 0   # per-connection index (aby DATA vždy čekalo na nový vzorek)
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
                        if core.init_gamepad():
                            started = core.start_compute_once()
                            logger  = start_dataloger_once()
                            _send_line(conn, "OK STARTED")
                        else:
                            _send_line(conn, "GAMEPAD NOT FOUND")

                    elif cmd == "DATA":
                        with core.cond:
                            core.cond.wait_for(lambda: core.stop_event.is_set() or core.msg_index > last_idx, timeout=0.5)
                            if core.stop_event.is_set():
                                _send_line(conn, "STOPPED")
                                continue
                            if core.latest_payload is None or core.msg_index == last_idx:
                                _send_line(conn, "NO DATA")
                                continue
                            last_idx = core.msg_index
                            payload = core.latest_payload  # str
                        _send_line(conn, payload)

                    elif cmd == "STOP":
                        _send_line(conn, "OK STOPPING GAMEPAD")
                        core.stop_all()
                        _send_line(conn, "OK STOPPED")

                    elif cmd == "EXIT":
                        send("BYE")
                        break

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
