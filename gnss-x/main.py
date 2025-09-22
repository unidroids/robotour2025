# main.py
import socket
import threading
import signal
from client import handle_client
from device import gnss_device   # <- přidáme import

HOST = "127.0.0.1"
PORT = 9006

client_threads = []
client_threads_lock = threading.Lock()
shutdown_flag = threading.Event()

def sigint_handler(signum, frame):
    print("\nSIGINT zachycen, ukoncuji GNSS server...")
    shutdown_flag.set()

def start_server():
    signal.signal(signal.SIGINT, sigint_handler)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen()
    print(f"🛰️ GNSS server nasloucha na {HOST}:{PORT}")

    try:
        while not shutdown_flag.is_set():
            server.settimeout(1.0)
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            print(f"📱 Klient pripojen: {addr}")
            t = threading.Thread(target=handle_client, args=(conn, addr, shutdown_flag), daemon=True)
            t.start()
            with client_threads_lock:
                client_threads.append(t)
    except Exception as e:
        print(f"Chyba serveru: {e}")
    finally:
        print("🛑 Ukoncuji server, zastavuji GNSS...")
        try:
            server.close()
        except Exception:
            pass

        # --- zastav GNSS vycitani ---
        gnss_device.stop()

        with client_threads_lock:
            for t in client_threads:
                t.join(timeout=1.0)
        print("✅ GNSS server korektne ukoncen.")

if __name__ == "__main__":
    start_server()