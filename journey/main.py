import socket
import threading
import signal
import sys

from workflow import journey_workflow, stop_workflow, demo_running
from util import log_event

HOST = '127.0.0.1'
PORT = 9004

shutdown_flag = threading.Event()

def handle_client(conn, addr):
    with conn:
        #conn.sendall(b"Welcome to Journey!\n")
        while not shutdown_flag.is_set():
            try:
                data = conn.recv(1024)
            except Exception:
                break
            if not data:
                break
            cmd = data.decode().strip().upper()
            log_event(f"CLIENT {addr}: {cmd}")
            if cmd == "PING":
                conn.sendall(b"PONG\n")
            elif cmd == "STOP":
                stop_workflow()
                conn.sendall(b"STOPPED\n")
            elif cmd == "DEMO":
                if demo_running.is_set():
                    conn.sendall(b"DEMO already running\n")
                else:
                    thread = threading.Thread(target=journey_workflow, args=(conn,))
                    thread.start()
            elif cmd == "LOG":
                for line in log_event.get_log()[-40:]:
                    conn.sendall((line+'\n').encode())
            else:
                conn.sendall(b"UNKNOWN COMMAND\n")

def server_main():
    print(f"Journey service listening on {HOST}:{PORT}")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        while not shutdown_flag.is_set():
            try:
                s.settimeout(1.0)
                conn, addr = s.accept()
                threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Server error: {e}")

def sigint_handler(signum, frame):
    print("\nSIGINT caught, shutting down Journey ...")
    shutdown_flag.set()
    stop_workflow()  # korektně ukončí DEMO, pošle STOP do služeb

if __name__ == "__main__":
    signal.signal(signal.SIGINT, sigint_handler)
    try:
        server_main()
    except Exception as e:
        print(f"Journey main exception: {e}")
    print("Journey service stopped.")

