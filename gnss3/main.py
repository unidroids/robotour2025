# main.py
import socket
import threading
import queue
import json
import sys
import signal

from gnss_serial import GnssSerialIO
from ubx_dispatcher import UbxDispatcher
from handlers.nav_hpposllh import NavHpposllhHandler
from handlers.nav_velned import NavVelNedHandler
from handlers.esf_ins import EsfInsHandler
from handlers.mon_sys import MonSysHandler
from handlers.mon_comms import MonCommsHandler
from handlers.dummy import DummyHandler

from builders import build_odo
from builders import build_perfect

from poller import RotatingPollerThread

SERVICE_PORT = 9006
SERIAL_DEVICE = '/dev/gnss1'

class GnssService:
    def __init__(self):
        self.gnss = None
        self.dispatcher = None
        self.bin_stream_fifo = queue.Queue(maxsize=30*2)
        self.fifo_lock = threading.Lock()
        self.hpposllh_handler = NavHpposllhHandler(self.bin_stream_fifo, self.fifo_lock)
        self.running = False
        self.poller = None

    def start(self):
        if self.running:
            return "ALREADY_RUNNING"
        self.gnss = GnssSerialIO(SERIAL_DEVICE)
        self.gnss.open()
        self.poller = RotatingPollerThread(self.gnss.send_ubx)
        self.poller.start()
        self.dispatcher = UbxDispatcher(self.gnss)
        self.dispatcher.register_handler(0x01, 0x05, DummyHandler()) # NAV-STATUS
        self.dispatcher.register_handler(0x01, 0x14, self.hpposllh_handler) # NAV-HPPOSLLH
        self.dispatcher.register_handler(0x01, 0x12, NavVelNedHandler(self.bin_stream_fifo,self.fifo_lock)) # NAV-VELNED
        #self.dispatcher.register_handler(0x10, 0x15, EsfInsHandler(self.bin_stream_fifo,self.fifo_lock)) # ESF-INS
        self.dispatcher.register_handler(0x10, 0x15, DummyHandler()) # ESF-INS
        self.dispatcher.register_handler(0x0a, 0x39, MonSysHandler()) # MON-SYS
        self.dispatcher.register_handler(0x0a, 0x36, MonCommsHandler()) # MON-COMMS
        self.dispatcher.start()
        self.running = True
        print("[SERVICE] STARTED")
        return "OK"

    def stop(self):
        if self.poller:
            self.poller.stop()
        if self.dispatcher:
            self.dispatcher.stop()
        if self.gnss:
            self.gnss.close()
        self.dispatcher = None
        self.gnss = None
        self.poller = None
        self.running = False
        print("[SERVICE] STOPPED")
        return "OK"

    def get_data_json(self):
        ctx = self.hpposllh_handler.get_last_context()
        if ctx:
            # převod timestamp na float (pro JSON serializaci)
            data = dict(ctx)
            if 'timestamp' in data:
                data['timestamp'] = float(data['timestamp'])
            return json.dumps(data)
        else:
            return "{}"

    def send_binary_stream(self, sock):
        # Posílá binární bloky z bin_stream_fifo do socketu (blokuje, dokud je klient připojen)
        try:
            while True:
                data = self.bin_stream_fifo.get(timeout=5.0)
                sock.sendall(data)
        except queue.Empty:
            pass
        except Exception as e:
            print(f"[SERVICE] Binary stream error: {e}")

# -- Jednoduchý socket server, jeden klient --
def client_thread(sock, addr, service: GnssService):
    #sock.settimeout(2.0)
    f = sock.makefile('rwb', buffering=0)
    print(f"[SERVER] Client connected: {addr}")
    try:
        while True:
            line = f.readline()
            if not line:
                break
            line = line.decode('utf-8').strip()
            #print(f"[SERVER] CMD: {line}")
            if line == "PING":
                f.write(b'PONG\n')
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
                json_data = service.get_data_json()
                f.write((json_data+'\n').encode('utf-8'))

            elif line.startswith("ODO "):
                try:
                    ubx = build_odo(cmd)   # builder dělá split, validaci, převod na binárku
                    self.gnss.send_ubx(ubx)
                    f.write(b"OK\n")
                except Exception as e:
                    f.write(b"ERROR: %s\n" % str(e).encode())

            elif line.startswith("PERFECT "):
                try:
                    ubx = build_perfect(cmd)
                    self.gnss.send_ubx(ubx)
                    f.write(b"OK\n")
                except Exception as e:
                    f.write(b"ERROR: %s\n" % str(e).encode())

            elif line == "GET_BINARY_STREAM":
                f.write(b'STREAM_READY\n')
                f.flush()
                service.send_binary_stream(sock)
                break  # ukončí po zavření streamu
            else:
                f.write(b'ERR UKNOWN COMMAND\n')
            f.flush()
    except Exception as e:
        print(f"[SERVER] Client error: {e}")
    finally:
        try:
            sock.close()
        except:
            pass
        print(f"[SERVER] Client disconnected: {addr}")

def main():
    service = GnssService()
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', SERVICE_PORT))
    sock.listen(1)
    print(f"[SERVER] GNSS Service listening on port {SERVICE_PORT}")

    # Zajistí Ctrl+C korektní ukončení služby
    def handle_sigint(signum, frame):
        print("Stopping GNSS service...")
        service.stop()
        sys.exit(0)
    signal.signal(signal.SIGINT, handle_sigint)

    while True:
        client_sock, addr = sock.accept()
        threading.Thread(target=client_thread, args=(client_sock, addr, service), daemon=True).start()

if __name__ == '__main__':
    main()
