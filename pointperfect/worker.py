# worker.py
import base64
import socket
import threading
import time
import os
from pathlib import Path
from dotenv import load_dotenv
from pointperfect_ntrip_client import NtripClient

env_path = Path(__file__).parent / ".env"
load_dotenv(env_path)

GNSS_HOST = "127.0.0.1"
GNSS_PORT = 9006

DEFAULT_SERVER = "ppntrip.services.u-blox.com"
DEFAULT_PORT = 2101
DEFAULT_MOUNTPOINT = "EU"   # můžeš upravit dle potřeby
DEFAULT_TLS = False

DEFAULT_USER = os.getenv("POINTPERFECT_USER", "")
DEFAULT_PASS = os.getenv("POINTPERFECT_PASS", "")


class PointPerfectWorker:
    def __init__(self):
        self.stop_event = threading.Event()
        self.msg_count = 0
        self.running = False
        self.stop_event = threading.Event()
        self.thread = None
        self._lock = threading.Lock()        

    def is_running(self) -> bool:
        return self.running

    def get_count(self) -> int:
        return self.msg_count

    def start(self):
        with self._lock:
            if self.running:
                print("▶️ PointPerfect již běží.")
                return
            self.stop_event.clear()
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
            self.running = True
            print("▶️ PointPerfect vlákno spuštěno")

    def stop(self):
        with self._lock:
            if not self.running:
                print("⏹️ PointPerfect není spuštěn.")
                return
            self.stop_event.set()
            if self.thread:
                self.thread.join(timeout=3.0)
            self.running = False
            print("⏹️ PointPerfect vlákno zastaveno")

    def _run(self):
        try:
            client = NtripClient(
                host=DEFAULT_SERVER,
                port=DEFAULT_PORT,
                user=DEFAULT_USER,
                password=DEFAULT_PASS,
                tls=DEFAULT_TLS,
            )
            client.start_stream(DEFAULT_MOUNTPOINT, self._handle_data)

            # čekej dokud není stop
            while not self.stop_event.is_set():
                time.sleep(1.0)

            client.stop_stream()

        except Exception as e:
            print(f"❌ Chyba ve PointPerfect workeru: {e}")
        finally:
            self.running = False

    def _handle_data(self, data: bytes):
        self.msg_count += 1
        ts = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        print(f"[{ts}] Přijato {len(data)} bajtů (#{self.msg_count})")

        try:
            payload = base64.b64encode(data).decode()
            cmd = f"PERFECT b64:{payload}\n"
            with socket.create_connection((GNSS_HOST, GNSS_PORT), timeout=2.0) as s:
                s.sendall(cmd.encode())
        except Exception as e:
            print(f"⚠️ Nelze poslat data do GNSS: {e}")
