# client.py – socket handler pro GNSS
import traceback
import json
from device import gnss_device
from ubx import build_esf_meas_ticks

import binascii
import base64

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
                        conn.sendall((json.dumps(state, indent=2, ensure_ascii=False) + "\n").encode())

                    elif cmd == "CALIBRATE":
                        # TODO: doplnit pro F9R (IMU), zatím placeholder
                        conn.sendall(b"NOT_IMPLEMENTED\n")

                    elif cmd == "DATA":
                        fix = gnss_device.get_fix()
                        conn.sendall((json.dumps(fix) + "\n").encode())

                    elif cmd.startswith("ODO "):
                        print(f"PERFECT")
                        # Formát: ODO %08X %08X %01X %08X %01X
                        try:
                            _, t_hex, l_hex, ld_hex, r_hex, rd_hex = cmd.split()
                            time_tag = int(t_hex, 16) & 0xFFFFFFFF
                            l_ticks  = int(l_hex, 16) & 0xFFFFFFFF
                            l_dir    = int(ld_hex, 16) & 0x1
                            r_ticks  = int(r_hex, 16) & 0xFFFFFFFF
                            r_dir    = int(rd_hex, 16) & 0x1

                            # vytvoří UBX-ESF-MEAS zprávu (2x wheel ticks)
                            msg = build_esf_meas_ticks(time_tag, l_ticks, l_dir, r_ticks, r_dir)

                            # vložíme do odesílací fronty GNSS zařízení
                            gnss_device.enqueue_raw(msg)

                            conn.sendall(b"OK\n")
                        except Exception as e:
                            conn.sendall(f"ERR ODO {e}\n".encode())

                    elif cmd.startswith("PERFECT "):
                        # PERFECT <payload>
                        # payload může být:
                        # 1) čistý hex řetězec (doporučeno), např. "D30100..."
                        # 2) base64 pokud začne prefixem "b64:"
                        try:
                            payload_str = cmd.split(" ", 1)[1].strip()
                            if payload_str.startswith("b64:"):
                                raw = base64.b64decode(payload_str[4:], validate=True)
                                print(f"Prijata base64 data: {len(raw)}")
                            else:
                                # odstraníme případné mezery
                                payload_str = payload_str.replace(" ", "")
                                raw = binascii.unhexlify(payload_str)
                                print(f"Prijata ascii data: {len(raw)}")
                            
                            gnss_device.enqueue_raw(raw)
                            conn.sendall(b"OK\n")
                        except Exception as e:
                            print(f"ERR PERFECT {e}\n".encode())
                            conn.sendall(f"ERR PERFECT {e}\n".encode())
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
