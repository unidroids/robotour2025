# client.py
import sys
import json
import traceback

def ensure_running(f, pilot):
    if not pilot.running:
        f.write(b"ERR: PILOT not started, use START first\n")
        f.flush()
        return False
    return True

def _handle_STATE(f, pilot):
    """Vrátí jednořádkový JSON se stavem pilota."""
    try:
        st = pilot.get_state()
        payload = json.dumps(st, separators=(",", ":"), ensure_ascii=False)
        f.write(f"OK STATE {payload}\n".encode("utf-8"))
    except Exception as e:
        f.write(f"ERROR STATE {type(e).__name__}: {e}\n".encode("utf-8"))
    finally:
        f.flush()

def client_thread(sock, addr, pilot):
    f = sock.makefile('rwb', buffering=0)
    print(f"[SERVER] Client connected: {addr}")
    try:
        while True:
            line = f.readline()
            if not line:
                break
            line = line.decode('utf-8').strip()
            try:
                if line == "PING":
                    f.write(b'PONG PILOT\n')

                elif line == "START":
                    res = pilot.start()
                    f.write((res+'\n').encode('utf-8'))

                elif line == "STOP":
                    res = pilot.stop()
                    f.write((res+'\n').encode('utf-8'))

                elif line == "STATE":
                    _handle_STATE(f, pilot)

                elif line == "EXIT":
                    f.write(b'GNSS-BYE\n')
                    f.flush()
                    break

                # --- doména pilot ---
                elif line.startswith("NAVIGATE"):
                    # NAVIGATE <start_lon> <start_lat> <end_lon> <end_lat> <end_radius>
                    if not ensure_running(f, pilot):
                        continue
                    try:
                        parts = line.split()
                        if len(parts) != 6:
                            f.write(b'ERR: NAVIGATE expects 5 arguments: <start_lon> <start_lat> <end_lon> <end_lat> <end_radius>\n')
                            f.flush()
                            continue
                        start_lon = float(parts[1])
                        start_lat = float(parts[2])
                        end_lon = float(parts[3])
                        end_lat = float(parts[4])
                        end_radius = float(parts[5])
                        # Pilot očekává (lon, lat); výpočty si uvnitř přemapují na (lat, lon)
                        start = (start_lon, start_lat)
                        goal = (end_lon, end_lat)
                        pilot.navigate(start, goal, end_radius)
                        f.write(b'OK NAVIGATE\n')
                    except Exception as e:
                        f.write(f"ERR: {e}\n".encode())
                    f.flush()

                # --- default response ---
                else:
                    f.write(b'ERR UNKNOWN COMMAND\n')
                    f.flush()

            except Exception as e:
                print(f"[CLIENT ERROR] {e}")
                traceback.print_exc()
                f.write(f"ERROR: {e}\n".encode())
                f.flush()
    except Exception as e:
        print(f"[SERVER] Client error: {e}")
        traceback.print_exc()
    finally:
        try:
            sock.close()
        except Exception:
            pass
        print(f"[SERVER] Client disconnected: {addr}")
