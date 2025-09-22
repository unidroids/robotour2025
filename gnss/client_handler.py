# client_handler.py
import sys
import json
from builders import build_odo, build_perfect, build_prio_on, build_prio_off

def ensure_gnss(f, service):
    if not service.gnss:
        f.write(b"ERR: GNSS not started, use START first\n")
        f.flush()
        return False
    return True

def client_thread(sock, addr, service):
    f = sock.makefile('rwb', buffering=0)
    print(f"[SERVER] Client connected: {addr}")
    try:
        while True:
            line = f.readline()
            if not line:
                break
            line = line.decode('utf-8').strip()
            print(f"[SERVER] Received command: {line}")
            try:
                if line == "PING":
                    f.write(b'PONG GNSS\n')
                elif line == "START":
                    res = service.start()
                    f.write((res+'\n').encode('utf-8'))
                elif line == "STOP":
                    res = service.stop()
                    f.write((res+'\n').encode('utf-8'))
                elif line == "EXIT":
                    f.write(b'EXITING\n')
                    service.stop()
                    sys.exit(0)
                elif line == "DATA":
                    if not ensure_gnss(f, service): continue
                    json_data = service.get_data_json()
                    f.write((json_data+'\n').encode('utf-8'))
                elif line.startswith("ODO "):
                    if not ensure_gnss(f, service): continue
                    ubx = build_odo(line)
                    service.gnss.send_ubx(ubx)
                    f.write(b"OK\n")
                elif line.startswith("PERFECT "):
                    if not ensure_gnss(f, service): continue
                    ubx = build_perfect(line)
                    service.gnss.send_ubx(ubx)
                    f.write(b"OK\n")
                elif line.strip() == "PRIO ON":
                    if not ensure_gnss(f, service): continue
                    ubx = build_prio_on()
                    service.gnss.send_ubx(ubx)
                    f.write(b"OK\n")
                elif line.strip() == "PRIO OFF":
                    if not ensure_gnss(f, service): continue
                    ubx = build_prio_off()
                    service.gnss.send_ubx(ubx)
                    f.write(b"OK\n")
                elif line == "GET_BINARY_STREAM":
                    if not ensure_gnss(f, service): continue
                    f.write(b'STREAM_READY\n')
                    f.flush()
                    service.send_binary_stream(sock)
                    break
                else:
                    f.write(b'ERR UNKNOWN COMMAND\n')
                f.flush()
            except Exception as e:
                print(f"[CLIENT ERROR] {e}")
                f.write(f"ERROR: {e}\n".encode())
                f.flush()
    except Exception as e:
        print(f"[SERVER] Client error: {e}")
    finally:
        try:
            sock.close()
        except:
            pass
        print(f"[SERVER] Client disconnected: {addr}")
