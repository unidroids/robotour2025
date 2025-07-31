from drive_serial import open_serial, close_serial, send_serial
from drive_client import handle_client
import socket
import threading
import signal
import sys

HOST = '127.0.0.1'
PORT = 9003

client_threads = []
client_threads_lock = threading.Lock()
shutdown_flag = False

def start_server():
    global shutdown_flag
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT))
    server.listen()
    print(f"🚦 robot-hoverboard server naslouchá na {HOST}:{PORT}")

    def signal_handler(sig, frame):
        global shutdown_flag
        print("\n🧯 Ctrl+C – ukončuji server a všechna spojení...")
        shutdown_flag = True
        server.close()
        with client_threads_lock:
            for t in client_threads:
                try:
                    t.join(timeout=2)
                except:
                    pass
        close_serial()
        print("🛑 Server ukončen.")
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        while not shutdown_flag:
            try:
                conn, addr = server.accept()
            except OSError:
                break
            t = threading.Thread(target=handle_client, args=(conn, addr))
            t.start()
            with client_threads_lock:
                client_threads.append(t)
    finally:
        close_serial()
        print("🛑 Server ukončen.")

if __name__ == "__main__":
    start_server()
