# client.py – obsluha klienta (Robotour 2025 cameras)
import traceback
import time
import socket



from worker import (
    shutdown_flag,
    start_camera_loop,
    stop_camera_loop,
    start_log_loop,
    stop_log_loop,
    start_qr_worker,
    qr_result,
    qr_ready,
    shutdown_flag,
)

def read_line(conn) -> str:
    """Čte jeden řádek ze socketu (ukončený \n)"""
    buffer = b""
    while not buffer.endswith(b"\n"):
        chunk = conn.recv(1)
        if not chunk:
            break
        buffer += chunk
    return buffer.decode("utf-8").strip().upper()

def handle_client(conn, addr):
    global qr_result, qr_ready, shutdown_flag
    try:
        with conn:
            buf = b""
            while not shutdown_flag.is_set():
                try:
                    cmd = read_line(conn)
                except socket.timeout:
                    continue
                if not cmd:
                    break

                print(f"📥 Příkaz od {addr}: {cmd}")

                # --- základní příkazy ---
                if cmd == "PING":
                    conn.sendall(b"PONG\n")

                elif cmd in ("RUN", "START"):
                    ok1 = start_camera_loop()
                    ok2 = start_log_loop()
                    msg = []
                    msg.append("LOOP OK" if ok1 else "LOOP ALREADY")
                    msg.append("LOG OK" if ok2 else "LOG ALREADY")
                    conn.sendall((" ".join(msg) + "\n").encode())

                elif cmd == "STOP":
                    stop_camera_loop()
                    stop_log_loop()
                    conn.sendall(b"STOPPED\n")

                elif cmd == "STATUS":
                    # TODO: může vracet reálný stav (RUNNING/IDLE)
                    conn.sendall(b"IDLE\n")

                elif cmd == "EXIT":
                    conn.sendall(b"BYE\n")
                    return

                elif cmd == "SHUTDOWN":
                    shutdown_flag.set()
                    conn.sendall(b"SHUTTING DOWN\n")
                    return

                # --- doménové příkazy ---
                elif cmd == "QR":
                    if not start_qr_worker():
                        conn.sendall(b"QR:LOOP NOT RUNNING\n")
                        continue

                    deadline = time.time() + 120
                    if qr_ready.wait(timeout=deadline - time.time()):
                        if qr_result:
                            conn.sendall(f"QR:{qr_result}\n".encode())
                            print(f"[client] QR FOUND: {qr_result}")
                        else:
                            conn.sendall(b"QR:NONE\n")
                            print(f"[client] QR NONE: {qr_result}")
                    else:
                        conn.sendall(b"QR:TIMEOUT\n")
                        print(f"[client] QR TIMEOUT")

                elif cmd == "LCAM":
                    conn.sendall(b"OK\n")
                elif cmd == "RCAM":
                    conn.sendall(b"OK\n")

                else:
                    conn.sendall(b"ERR Unknown cmd\n")

    except Exception as e:
        print(f"❌ Chyba klienta {addr}: {e}")
        traceback.print_exc()
    finally:
        print(f"🔌 Odpojeno: {addr}")
