#!/usr/bin/env python3
# TCP server: 127.0.0.1:9005 – PING, START, DATA, STOP + Ctrl+C
import socket, threading, signal, sys
from gamepad_client import handle_client
from gamepad_control import set_server_socket, shutdown_event, close_all_clients
from gamepad_core import stop_all

HOST = "127.0.0.1"
PORT = 9005

def serve_forever():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen()
    set_server_socket(server)
    print(f"[SERVER] naslouchá na {HOST}:{PORT} (Ctrl+C pro ukončení)")

    def on_sigint(sig, frame):
        print("\n[SERVER] Ctrl+C -> STOP")
        shutdown_event.set()
        try: server.close()
        except: pass
        stop_all()
        close_all_clients()
        sys.exit(0)

    signal.signal(signal.SIGINT, on_sigint)

    try:
        while not shutdown_event.is_set():
            try:
                conn, addr = server.accept()
            except OSError:
                break  # socket zavřen (STOP / Ctrl+C)
            t = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
            t.start()
    finally:
        try: server.close()
        except: pass
        close_all_clients()
        print("[SERVER] serve_forever STOP")

if __name__ == "__main__":
    serve_forever()
