from drive_serial import open_serial, close_serial, send_serial, start_serial_reader, stop_serial_reader
import threading
import socket
import re
import time
from collections import deque

shutdown_flag = False

def clamp(val, minval, maxval):
    return max(minval, min(maxval, val))

def pwm_left_to_serial_byte(axis_1):
    # Negace osy podle vzoru!
    val = clamp(int(round(axis_1 * 30)), -30, 30)
    return val + 158

def pwm_right_to_serial_byte(axis_3):
    val = clamp(int(round(axis_3 * 30)), -30, 30)
    return val + 219


# Přesná validace ODO řádku z hoverboardu: "%08X %08X %01X %08X %01X"
HEX_ODO_RE = re.compile(
    r'^(?P<T>[0-9A-Fa-f]{8})\s+(?P<L>[0-9A-Fa-f]{8})\s+(?P<LD>[0-9A-Fa-f]{1})\s+(?P<R>[0-9A-Fa-f]{8})\s+(?P<RD>[0-9A-Fa-f]{1})$'
)

GNSS_HOST = '127.0.0.1'
GNSS_PORT = 9006


class OdoForwarder:
    """
    Rychlý přeposílač ODO → GNSS:
    - reader vlákno jen ENQUEUE (nezdržuje se sítí)
    - vlastní vlákno s frontou (deque) posílá do GNSS
    - krátká lhůta na ACK (20 ms). Pokud nic nepřijde, neblokujeme.
    """
    def __init__(self, host, port, max_queue=50):
        self.addr = (host, port)
        self.sock = None
        self.lock = threading.Lock()
        self.q = deque(maxlen=max_queue)
        self.stop_ev = threading.Event()
        self.th = None
        self._last_conn_err_ts = 0.0

    def start(self):
        if self.th and self.th.is_alive():
            return
        self.stop_ev.clear()
        self.th = threading.Thread(target=self._loop, name="odo-forwarder", daemon=True)
        self.th.start()
        print("🚀 ODO forwarder spuštěn.")

    def stop(self):
        if self.th and self.th.is_alive():
            self.stop_ev.set()
            print("🛑 ODO forwarder stop požadavek.")

    def enqueue(self, payload_line: str):
        self.q.append(payload_line)

    # --- vnitřní pomocné ---

    def _ensure(self) -> bool:
        if self.sock:
            return True
        # omez reconnect spam (max 1 pokus / 500 ms)
        now = time.monotonic()
        if now - self._last_conn_err_ts < 0.5:
            return False
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(0.2)  # rychlá konexe
            s.connect(self.addr)
            s.settimeout(0.02) # velmi krátké čekání na odpověď
            self.sock = s
            print(f"🔗 GNSS připojeno: {self.addr}")
            return True
        except Exception as e:
            self._last_conn_err_ts = now
            print(f"⚠️  GNSS nepřístupné {self.addr}: {e}")
            self.sock = None
            return False

    def _send_once(self, payload_line: str):
        """Odešle 'ODO <payload>' a velmi krátce poslouchá na odpověď."""
        if not self._ensure():
            return
        try:
            self.sock.sendall(f"ODO {payload_line}\n".encode('ascii'))

            # pokus o 1-řádkovou odpověď s velmi krátkým deadlinem (~20 ms)
            data = b""
            deadline = time.monotonic() + 0.02
            while time.monotonic() < deadline:
                try:
                    chunk = self.sock.recv(256)
                    if not chunk:
                        break
                    data += chunk
                    if b"\n" in data:
                        break
                except (socket.timeout, BlockingIOError):
                    break

            if data:
                reply = data.decode('ascii', errors='replace').strip()
                if reply.startswith("ERR") or "ERR ODO" in reply:
                    print(f"❌ GNSS odpověď: {reply}")
            # když nic nepřišlo → neblokujeme, jedeme dál

        except Exception as e:
            print(f"⚠️  GNSS chyba při odesílání ODO: {e}")
            try:
                if self.sock:
                    self.sock.close()
            finally:
                self.sock = None

    def _loop(self):
        while not self.stop_ev.is_set():
            if self.q:
                line = self.q.popleft()
                self._send_once(line)
            else:
                time.sleep(0.001)


_forwarder = OdoForwarder(GNSS_HOST, GNSS_PORT)


def _on_serial_line(line: str):
    """
    Přijde např.: "0002EAE0 0000A1B2 1 0000C3D4 0"
    Pokud sedí formát, vloží se do fronty a ihned vrátíme (reader nečeká na síť).
    """
    u = line.strip()
    if HEX_ODO_RE.match(u):
        # hoverboard tiskne %X => už uppercase; kdyby ne, sjednotíme:
        _forwarder.enqueue(u.upper())
    # ostatní řádky ignorujeme (diagnostika apod.)


def handle_client(conn, addr):
    print(f"📡 Klient připojen: {addr}")
    global shutdown_flag

    def send(msg):
        try:
            conn.sendall((msg + '\n').encode())
        except:
            pass

    try:
        with conn:
            while not shutdown_flag:
                data = conn.recv(1024)
                if not data:
                    break
                cmd_line = data.decode().strip()
                parts = cmd_line.split()
                if not parts:
                    continue
                cmd = parts[0].upper()

                if cmd == "PING":
                    send("PONG")

                elif cmd == "START":
                    open_serial()
                    start_serial_reader(_on_serial_line)  # ⚡ začne číst a okamžitě enqueovat
                    _forwarder.start()                    # ⚡ začne posílat do GNSS
                    send("OK")

                elif cmd == "STOP":
                    stop_serial_reader()
                    close_serial()
                    _forwarder.stop()
                    send("OK")

                elif cmd == "PWM":
                    # Očekáváme hodnoty v rozsahu -100..100 (jako v původním zadání)
                    if len(parts) == 3:
                        try:
                            axis_1 = float(parts[1]) / 100.0  # převod na -1..1
                            axis_3 = float(parts[2]) / 100.0
                            left_motor = pwm_left_to_serial_byte(axis_1)
                            right_motor = pwm_right_to_serial_byte(axis_3)
                            send_serial(bytes([left_motor]))
                            send_serial(bytes([right_motor]))
                            print(f"PWM {axis_1:.2f} {axis_3:.2f} => {left_motor} {right_motor}")
                            send("OK")
                        except Exception as e:
                            send(f"ERR {e}")
                    else:
                        send("ERR Param count")

                elif cmd == "BREAK" or cmd == "B":
                    # Posílá "střed" (zastavení) pro obě kola
                    send_serial(bytes([158]))
                    send_serial(bytes([219]))
                    send("OK")

                elif cmd == "EXIT":
                    send("BYE")
                    break

                else:
                    send("ERR Unknown cmd")
    except Exception as e:
        print(f"Chyba: {e}")
    finally:
        try:
            conn.close()
        except:
            pass
        print(f"🔌 Odpojeno: {addr}")
