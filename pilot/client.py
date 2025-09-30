# client_handler.py
import sys

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
                    f.write(b'[SERVER] Navigation points recived.\n')
                    f.flush()
                    #TODO implement
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
