# client.py
import sys
import json
import traceback
import socket

from service import FusionService


def ensure_running(f, fusion):
    if not fusion.running:
        f.write(b"ERR: PILOT not started, use START first\n")
        f.flush()
        return False
    return True

def client_thread(sock:socket.socket, addr, fusion : FusionService):
    f = sock.makefile('rwb', buffering=0)
    print(f"[SERVER] Client connected: {addr}")
    try:
        while True:
            try:
                line = f.readline()
            except ConnectionResetError as e:
                break
            if not line:
                break
            line = line.decode('utf-8').strip()
            try:
                # --- doména fusion ---

                if line == "LIDAR": # data from gnss
                    payload = f.read(fusion.LIDAR_MESSAGE_LENGHT)
                    if not ensure_running(f, fusion): continue
                    fusion.on_lidar_data(payload)
                    f.write(b'OK\n')
                
                elif line == "DRIVE": # data from drive
                    payload = f.readline() # text base data from drive
                    if not ensure_running(f, fusion): continue
                    fusion.on_drive_data(payload)
                    f.write(b'OK\n')

                elif line == "HEADING": # data from drive
                    payload = f.readline() # text base data from drive
                    if not ensure_running(f, fusion): continue
                    fusion.on_heading_data(payload)
                    f.write(b'OK\n')

                elif line == "GNSS": # data from gnss
                    payload = f.read(fusion.GNSS_MESSAGE_LENGHT) #binary data from gnss
                    if not ensure_running(f, fusion): continue
                    fusion.on_gnss_data(payload)
                    f.write(b'OK\n')
                
                elif line == "CAMERA": # data from camera
                    payload = f.read(fusion.CAMERA_MESSAGE_LENGHT)
                    if not ensure_running(f, fusion): continue
                    fusion.on_camera_data(payload)
                    f.write(b'OK\n')


                # --- standard API ---

                elif line == "PING":
                    f.write(b'PONG FUSION\n')

                elif line == "RESTART":
                    res = fusion.restart()
                    f.write((res+'\n').encode('utf-8'))

                elif line == "STATE":
                    st = fusion.get_state()
                    payload = json.dumps(st, separators=(",", ":"), ensure_ascii=False)
                    f.write(f"OK STATE {payload}\n".encode("utf-8"))

                elif line == "EXIT":
                    f.write(b'OK-FUSION-BYE\n')
                    f.flush()
                    break
                
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
