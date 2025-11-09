# main.py
import socket
import signal
import threading
import sys

from fusion import Fusion
from client import client_thread

SERVICE_PORT = 9009

def main():
    fusion = Fusion()
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', SERVICE_PORT))
    sock.listen(1)
    print(f"[SERVER] Service FUSION listening on port {SERVICE_PORT}")

    def handle_sigint(signum, frame):
        print("[SERVER] Stopping FUSION service...")
        fusion.stop()
        sys.exit(0)
    signal.signal(signal.SIGINT, handle_sigint)

    while True:
        client_sock, addr = sock.accept()
        threading.Thread(target=client_thread, args=(client_sock, addr, fusion), daemon=True).start()

if __name__ == '__main__':
    main()

