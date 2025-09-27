# worker.py
import base64
import socket
import threading
import time
from time import monotonic
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

GGA_INTERVAL_SEC = 10  # každých 10 sekund posílat GGA

class PointPerfectWorker:
    def __init__(self):
        self.stop_event = threading.Event()
        self.msg_count = 0
        self.running = False
        self.stop_event = threading.Event()
        self.thread = None
        self._lock = threading.Lock()        
        self.client = None
        self._streaming = False

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
            if self.client:
                try:
                    self.client.stop_stream()
                except Exception:
                    pass
                self.client = None
            if self.thread:
                self.thread.join(timeout=3.0)
            self.running = False
            print("⏹️ PointPerfect vlákno zastaveno")

    def _run(self):
        try:

            # 1. Čekání na validní GGA před startem streamu
            while not self.stop_event.is_set():
                gga = self._get_gga_sentence(GNSS_HOST, GNSS_PORT)
                if gga and self._is_gga_valid(gga):
                    print(f"✅ Validní GGA k dispozici")
                    break
                else:
                    print("⏳ Čekám na validní GGA...")
                time.sleep(1.0)
            if self.stop_event.is_set():
                return

            # 2. Start streamu – propojí klienta se serverem, začne přijímat data
            print(f"Startuji stream ({DEFAULT_SERVER}:{DEFAULT_PORT} / {DEFAULT_MOUNTPOINT})...")
            self.client = NtripClient(
                host=DEFAULT_SERVER,
                port=DEFAULT_PORT,
                user=DEFAULT_USER,
                password=DEFAULT_PASS,
                tls=DEFAULT_TLS,
            )
            self.client.start_stream(DEFAULT_MOUNTPOINT, self._handle_data)
            self._streaming = True
            print("✅ NTRIP stream spuštěn.")
            time.sleep(5.0)
            self.client.send_gga(gga)  # Pošleme první GGA hned po startu streamu
            print(f"➡️  GGA odeslána: {gga.strip()}")    

            # 3. Smyčka: každých 10s posílej GGA, pokud je socket připojen
            last_gga_sent_ts = time.monotonic()
            while not self.stop_event.is_set():
                now = monotonic()
                if now - last_gga_sent_ts >= GGA_INTERVAL_SEC:
                    gga = self._get_gga_sentence(GNSS_HOST, GNSS_PORT)
                    if gga and self._is_gga_valid(gga):
                        self.client.send_gga(gga)
                        print(f"➡️  GGA odeslána: {gga.strip()}")
                    else:
                        print("⚠️ GGA není validní, neodesílám.")
                    last_gga_sent_ts = monotonic()
                time.sleep(1.0)


        except Exception as e:
            print(f"❌ Chyba ve PointPerfect workeru: {e}")
        finally:
            try:
                if self.client:
                    self.client.stop_stream()
                    print("✅ NTRIP stream zastaven.")
            except Exception:
                pass

            self.client = None
            self._streaming = False

            self.running = False
            print("🛑 PointPerfect worker ukončen.")

    def _get_gga_sentence(self, host, port):
        try:
            with socket.create_connection((host, port), timeout=1.5) as s:
                s.sendall(b"GGA\n")
                s.settimeout(1.5)
                data = s.recv(2048)
                if not data:
                    return None
                line = data.decode("ascii", errors="ignore").strip()
                if not line:
                    return None
                line = line.splitlines()[0]
                if "GGA" not in line:
                    return None
                if not (line.endswith("\r\n") or line.endswith("\n") or line.endswith("\r")):
                    line = line + "\r\n"
                return line
        except (socket.timeout, ConnectionRefusedError, OSError):
            return None

    @staticmethod
    def _is_gga_valid(gga_sentence: str) -> bool:
        """
        Minimální validace:
        - fix quality (pole 7) musí být > 0
        - počet satelitů (pole 8) >= 4

        NMEA GGA:
        $xxGGA,UTC,lat,N,lon,E,fix,numsats,hdop,alt,M,...*CS
                0   1    2  3   4  5  6   7       8    9 10
        """
        try:
            body = gga_sentence.strip().split('*')[0]
            if not body.startswith('$') or 'GGA' not in body:
                return False
            
            body = body[1:]  # Odstraníme '$'
            parts = body.split(',')
            if len(parts) < 8:
                return False

            fix_str = parts[6]
            numsats_str = parts[7]

            fix = int(fix_str) if fix_str.isdigit() else 0
            numsats = int(numsats_str) if numsats_str.isdigit() else 0

            # Validní pouze když fix > 0 (1=GPS fix, 2=DGPS fix, 4=RTK fix, …)
            return fix > 0 and numsats >= 4
        except Exception:
            return False

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
