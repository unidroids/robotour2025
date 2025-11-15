# main.py
import socket
import signal
import threading
import sys

from service import UnicoreService
from client import client_thread

SERVICE_PORT = 9010

def main():
    service = UnicoreService()
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', SERVICE_PORT))
    sock.listen(1)
    print(f"[SERVER] Unicore Service listening on port {SERVICE_PORT}")

    def handle_sigint(signum, frame):
        print("Stopping GNSS service...")
        service.stop()
        sys.exit(0)
    signal.signal(signal.SIGINT, handle_sigint)
    signal.signal(signal.SIGTERM, handle_sigint)

    while True:
        client_sock, addr = sock.accept()
        threading.Thread(target=client_thread, args=(client_sock, addr, service), daemon=True).start()

if __name__ == '__main__':
    main()
