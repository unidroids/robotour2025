# client_handler.py
import sys
import json
import socket
from service import UnicoreService


def ensure_gnss(f:socket.SocketIO, service: UnicoreService):
    if not service.running:
        f.write(b"ERR: HEADING service not started, use START first\n")
        f.flush()
        return False
    return True

def client_thread(sock:socket.socket, addr, service: UnicoreService):
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
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
                    f.write(b'PONG HEADING\n')
                
                elif line == "START":
                    res = service.start()
                    f.write((res+'\n').encode('utf-8'))
                
                elif line == "STOP":
                    res = service.stop()
                    f.write((res+'\n').encode('utf-8'))
                
                elif line == "EXIT":
                    f.write(b'OK-HEADING-BYE\n')
                    f.flush()
                    break

                elif line == "STATE":
                    st = service.get_state() 
                    print(st)
                    f.write((json.dumps(st, separators=(",",":"))+ "\n").encode("utf-8"))
                    
                
                elif line == "HEADING":
                    if not ensure_gnss(f, service): continue
                    sent = service.get_heading() 
                    if sent is not None:
                        f.write(sent)
                    else:
                        f.write(b'')          
                
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
