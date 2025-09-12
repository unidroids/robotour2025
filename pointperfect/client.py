# client.py
import traceback
from worker import PointPerfectWorker

# Globální worker pro PointPerfect stream
worker = PointPerfectWorker()


def handle_client(conn, addr, shutdown_flag):
    try:
        with conn:
            buf = b""
            while not shutdown_flag.is_set():
                data = conn.recv(1024)
                if not data:
                    break
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    cmd = line.decode(errors="ignore").strip()
                    if not cmd:
                        continue

                    # --- základní příkazy ---
                    if cmd == "PING":
                        conn.sendall(b"PONG\n")

                    elif cmd == "START":
                        if not worker.is_running():
                            worker.start()
                        conn.sendall(b"OK\n")

                    elif cmd == "STOP":
                        if worker.is_running():
                            worker.stop()
                        conn.sendall(b"OK\n")

                    elif cmd == "STATUS":
                        status = "RUNNING" if worker.is_running() else "IDLE"
                        conn.sendall(f"{status} {worker.get_count()}\n".encode())

                    elif cmd == "EXIT":
                        conn.sendall(b"BYE\n")
                        return

                    else:
                        conn.sendall(b"ERR Unknown cmd\n")

    except Exception as e:
        print(f"Chyba klienta {addr}: {e}")
        traceback.print_exc()
    finally:
        print(f"🔌 Odpojeno: {addr}")
