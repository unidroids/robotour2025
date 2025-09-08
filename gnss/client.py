# client.py – socket handler pro GNSS
import traceback
import json
from device import gnss_device

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

                    if cmd == "PING":
                        conn.sendall(b"PONG\n")

                    elif cmd == "START":
                        ok = gnss_device.start()
                        conn.sendall(b"OK\n" if ok else b"ERROR start\n")

                    elif cmd == "STOP":
                        gnss_device.stop()
                        conn.sendall(b"OK\n")

                    elif cmd == "STATE":
                        state = gnss_device.get_state()
                        conn.sendall((json.dumps(state) + "\n").encode())

                    elif cmd == "CALIBRATE":
                        # TODO: doplnit pro F9R (IMU), zatím placeholder
                        conn.sendall(b"NOT_IMPLEMENTED\n")

                    elif cmd == "DATA":
                        fix = gnss_device.get_fix()
                        conn.sendall((json.dumps(fix) + "\n").encode())

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
