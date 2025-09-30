# client_handler.py
import sys
import traceback

def ensure_running(f, pilot):
    if not pilot.running:
        f.write(b"ERR: PILOT not started, use START first\n")
        f.flush()
        return False
    return True

def client_thread(sock, addr, pilot):
    f = sock.makefile('rwb', buffering=0)
    print(f"[SERVER] Client connected: {addr}")
    try:
        while True:
            line = f.readline()
            if not line:
                break
            line = line.decode('utf-8').strip()
            #print(f"[SERVER] Received command: {line}")
            try:
                if line == "PING":
                    f.write(b'PONG PILOT\n')
                elif line == "START":
                    res = pilot.start()
                    f.write((res+'\n').encode('utf-8'))
                elif line == "STOP":
                    res = pilot.stop()
                    f.write((res+'\n').encode('utf-8'))
                elif line == "EXIT":
                    f.write(b'GNSS-BYE\n')
                    f.flush()
                    break
                # --- doména pilot ---
                elif line.startswith("NAVIGATE"):
                    if not ensure_running(f, pilot): continue
                    try:
                        parts = line.split()
                        if len(parts) != 6:
                            f.write(b'ERR: NAVIGATE expects 5 arguments\n')
                            f.flush()
                            continue
                        # NAVIGATE <start lon> <start lat> <end lon> <end lat> <end radius>
                        start_lat = float(parts[1])
                        start_lon = float(parts[2])
                        end_lat = float(parts[3])
                        end_lon = float(parts[4])
                        end_radius = float(parts[5])
                        start = (start_lon, start_lat)
                        goal = (end_lon, end_lat)
                        pilot.navigate(start, goal, end_radius)
                        f.write(b'[SERVER] Navigation points received.\n')
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
        except:
            pass
        print(f"[SERVER] Client disconnected: {addr}")
