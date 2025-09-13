# client.py – obsluha klienta (Robotour 2025 cameras)
import traceback
import time
import socket
import worker  # << důležité: import celého modulu

def read_line(conn) -> str:
    buffer = b""
    while not buffer.endswith(b"\n"):
        chunk = conn.recv(1)
        if not chunk:
            break
        buffer += chunk
    return buffer.decode("utf-8").strip().upper()

def handle_client(conn, addr):
    try:
        conn.settimeout(2.0)
        with conn:
            while not worker.shutdown_flag.is_set():
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
                    ok1 = worker.start_camera_loop()
                    ok2 = worker.start_log_loop()
                    msg = []
                    msg.append("LOOP OK" if ok1 else "LOOP ALREADY")
                    msg.append("LOG OK" if ok2 else "LOG ALREADY")
                    conn.sendall((" ".join(msg) + "\n").encode())

                elif cmd == "STOP":
                    worker.stop_camera_loop()
                    worker.stop_log_loop()
                    conn.sendall(b"STOPPED\n")

                elif cmd == "STATUS":
                    # TODO: vrať reálný stav (RUNNING/IDLE)
                    conn.sendall(b"IDLE\n")

                elif cmd == "EXIT":
                    conn.sendall(b"BYE\n")
                    return

                elif cmd == "SHUTDOWN":
                    worker.shutdown_flag.set()
                    conn.sendall(b"SHUTTING DOWN\n")
                    return

                # --- doménové příkazy ---
                elif cmd == "QR":
                    if not worker.start_qr_worker():
                        conn.sendall(b"QR:LOOP NOT RUNNING\n")
                        continue

                    print("🧾 QR worker spuštěn (čekám na výsledek)")
                    deadline = time.time() + 120
                    if worker.qr_ready.wait(timeout=deadline - time.time()):
                        # ČTI POD ZÁMKEM a z modulu
                        with worker.qr_lock:
                            result = worker.qr_result
                        if result:
                            conn.sendall(f"QR:{result}\n".encode())
                            print(f"[client] QR FOUND: {result}")
                        else:
                            conn.sendall(b"QR:NONE\n")
                            print("[client] QR NONE (žádný kód)")
                    else:
                        conn.sendall(b"QR:TIMEOUT\n")
                        print("[client] QR TIMEOUT")

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
