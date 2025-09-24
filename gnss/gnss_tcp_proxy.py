import socket
import threading
import time
from collections import deque

UBX_SYNC1 = 0xB5
UBX_SYNC2 = 0x62

class GnssTcpProxyServer:
    def __init__(self, host='0.0.0.0', port=5000, fifo_size=60, gnss_writer=None, gnss_source=None):
        """
        :param gnss_writer: callable(msg: bytes) – zapisuje UBX zprávy do GNSS modulu
        :param gnss_source: callable(callback) – registruje callback, který bude volán s každou UBX zprávou z GNSS zařízení
        """
        self.host = host
        self.port = port
        self.fifo_size = fifo_size
        self.fifo = deque(maxlen=fifo_size)
        self.fifo_lock = threading.Lock()
        self.gnss_writer = gnss_writer
        self.gnss_source = gnss_source

        self.server_sock = None
        self.client_sock = None
        self.client_addr = None
        self.client_connected = threading.Event()

        self.running = False
        self._fifo_stop = threading.Event()

    def start(self):
        # Přihlášení na příjem GNSS zpráv (musí volat vždy, i když není klient)
        if self.gnss_source:
            self.gnss_source(self._on_gnss_message)
        else:
            print("[GNSS TCP PROXY] Chybí gnss_source – nebude přijímat zprávy z GNSS!")

        self.running = True
        server_thread = threading.Thread(target=self._server_loop, daemon=True)
        server_thread.start()
        print(f"[GNSS TCP PROXY] Listening on {self.host}:{self.port}")

    def _on_gnss_message(self, ubx_msg: bytes):
        """Callback – vždy celá UBX zpráva (včetně hlavičky B5 62 ... CRC)"""
        with self.fifo_lock:
            self.fifo.append(ubx_msg)

    def _server_loop(self):
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((self.host, self.port))
        self.server_sock.listen(1)

        while self.running:
            try:
                client_sock, client_addr = self.server_sock.accept()
                if self.client_connected.is_set():
                    print(f"[GNSS TCP PROXY] Připojení odmítnuto, už je aktivní klient z {self.client_addr}")
                    client_sock.close()
                    continue
                print(f"[GNSS TCP PROXY] Klient připojen: {client_addr}")
                self.client_sock = client_sock
                self.client_addr = client_addr
                self.client_connected.set()

                # Vždy po připojení klienta spustíme reader i writer vlákno
                reader_thread = threading.Thread(target=self._client_reader, daemon=True)
                writer_thread = threading.Thread(target=self._client_writer, daemon=True)
                reader_thread.start()
                writer_thread.start()

                reader_thread.join()
                writer_thread.join()

            except Exception as e:
                print(f"[GNSS TCP PROXY] Výjimka v serveru: {e}")

            self.client_connected.clear()
            if self.client_sock:
                try:
                    self.client_sock.close()
                except Exception:
                    pass
                self.client_sock = None
            print("[GNSS TCP PROXY] Klient odpojen.")

    def _client_reader(self):
        """Přijímá data od klienta, dekóduje UBX zprávy a předává do GNSS"""
        buffer = b''
        sock = self.client_sock
        while self.client_connected.is_set():
            try:
                data = sock.recv(4096)
                if not data:
                    break
                buffer += data
                while True:
                    # Najdi sync bytes
                    idx = buffer.find(bytes([UBX_SYNC1, UBX_SYNC2]))
                    if idx < 0 or len(buffer) - idx < 6:
                        buffer = buffer[idx:] if idx >= 0 else b''
                        break
                    if idx > 0:
                        # Zahodit junk před sync
                        print(f"[GNSS TCP PROXY] Warning: zahazuji {idx} bajtů před UBX sync")
                        buffer = buffer[idx:]
                    # UBX zpráva: 2B sync + 2B msg + 2B len + len(payload) + 2B crc
                    if len(buffer) < 6:
                        break
                    _, _, msg_class, msg_id, length_lo, length_hi = buffer[:6]
                    payload_len = length_lo + (length_hi << 8)
                    full_msg_len = 6 + payload_len + 2
                    if len(buffer) < full_msg_len:
                        break
                    ubx_msg = buffer[:full_msg_len]
                    buffer = buffer[full_msg_len:]
                    # Můžeš ověřit CRC zde – pokud chceš (volitelně)
                    if self.gnss_writer:
                        self.gnss_writer(ubx_msg)
                    else:
                        print("[GNSS TCP PROXY] Chybí gnss_writer – nemohu poslat zprávu do GNSS!")
            except Exception as e:
                print(f"[GNSS TCP PROXY] Reader error: {e}")
                break

    def _client_writer(self):
        """Posílá FIFO zprávy klientovi (z GNSS portu, kompletní UBX balíky)"""
        sock = self.client_sock
        while self.client_connected.is_set():
            try:
                msg = None
                with self.fifo_lock:
                    if self.fifo:
                        msg = self.fifo.popleft()
                if msg:
                    try:
                        sock.sendall(msg)
                    except Exception as e:
                        print(f"[GNSS TCP PROXY] Writer error: {e}")
                        break
                else:
                    time.sleep(0.005)  # micro-sleep pro idle
            except Exception as e:
                print(f"[GNSS TCP PROXY] Writer loop error: {e}")
                break

    def stop(self):
        self.running = False
        self.client_connected.clear()
        if self.server_sock:
            try:
                self.server_sock.close()
            except Exception:
                pass
        if self.client_sock:
            try:
                self.client_sock.close()
            except Exception:
                pass
        print("[GNSS TCP PROXY] Stopped.")

# === Příklad použití (místo gnss_writer a gnss_source dosaď tvé konkrétní implementace) ===

if __name__ == "__main__":
    # Placeholder pro zápis do GNSS zařízení
    def my_gnss_writer(ubx_msg: bytes):
        print(f"→ GNSS: {ubx_msg.hex()[:32]}... [{len(ubx_msg)} B]")

    # Placeholder pro příjem zpráv z GNSS zařízení – volat server._on_gnss_message(zprava) vždy při přijetí celé UBX
    def my_gnss_source(callback):
        # Simulace: generuj každou sekundu fake UBX zprávu
        import threading
        import random
        def fake_gnss():
            while True:
                fake = bytes([0xB5,0x62,0x01,0x07,0x1C,0x00] + [random.randint(0,255) for _ in range(28)] + [0,0])
                callback(fake)
                time.sleep(1)
        threading.Thread(target=fake_gnss, daemon=True).start()

    proxy = GnssTcpProxyServer(gnss_writer=my_gnss_writer, gnss_source=my_gnss_source)
    proxy.start()

    # Pro demo běž neomezeně (Ctrl+C kill)
    while True:
        time.sleep(1)
