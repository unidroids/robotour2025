# client_handler.py
import sys
import json
from builders import build_odo
from builders import build_perfect
from builders import build_prio_on, build_prio_off


def ensure_gnss(f, service):
    if not service.gnss:
        f.write(b"ERR: GNSS not started, use START first\n")
        f.flush()
        return False
    return True

def client_thread(sock, addr, service):
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
                    f.write(b'PONG GNSS\n')
                elif line == "START":
                    res = service.start()
                    f.write((res+'\n').encode('utf-8'))
                elif line == "STOP":
                    res = service.stop()
                    f.write((res+'\n').encode('utf-8'))
                elif line == "EXIT":
                    f.write(b'GNSS-BYE\n')
                    f.flush()
                    break
                elif line == "GGA":
                    sent = service.get_gga() 
                    if sent is not None:
                        f.write(sent)
                    else:
                        f.write(b'')                    
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
                    try:
                        while True:
                            if not service.wait_for_update(timeout=1.0):
                                continue
                            f.write(service.get_data_binary())
                            f.flush()
                            # Volitelně: přerušení na základě dalšího příkazu od klienta
                            # Pokud chceš, aby klient mohl ukončit stream, můžeš zde číst další řádky:
                            # break pokud přijde "STOP_STREAM" nebo socket zavřen
                            # Příklad:
                            # if f.peek(1):  # pokud je něco v bufferu od klienta
                            #     cmd = f.readline().decode().strip()
                            #     if cmd == "STOP_STREAM": break
                    except Exception as e:
                        print(f"[CLIENT STREAM ERROR] {e}")
                    break  # ukonči po streamu
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
