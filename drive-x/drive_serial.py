import serial
import threading
import time

SERIAL_PORT = '/dev/howerboard'
SERIAL_BAUD = 115200

serial_lock = threading.Lock()
serial_conn = None

# --- Stav řádkového readeru (low-latency) ---
_reader_thread = None
_reader_stop = threading.Event()
_line_handler = None


def open_serial():
    """Otevře sériový port a pošle hlavičku (13). Non-blocking read pro nízkou latenci."""
    global serial_conn
    if serial_conn is None or not serial_conn.is_open:
        serial_conn = serial.Serial(
            port=SERIAL_PORT,
            baudrate=SERIAL_BAUD,
            timeout=0,       # ⚡ neblokující čtení (latence ~1 ms ve smyčce)
            write_timeout=0, # neblokující zápis
            dsrdtr=False,
            rtscts=False
        )
        serial_conn.setDTR(False)
        serial_conn.setRTS(False)
        print(f"Serial otevřen na {SERIAL_PORT}")

        # Po otevření pošli hlavičku (bajt 13)
        with serial_lock:
            serial_conn.write(bytes([13]))
        print("Odeslána hlavička (13)")
    else:
        print("Serial už je otevřený.")


def close_serial():
    """Pošle ukončovací bajt (27) a korektně zavře port."""
    global serial_conn
    if serial_conn and serial_conn.is_open:
        try:
            with serial_lock:
                serial_conn.write(bytes([27]))
            print("Odeslán ukončovací bajt (27)")
        except Exception:
            pass
        try:
            serial_conn.close()
        finally:
            print("Serial uzavřen.")
    else:
        print("Serial není otevřen.")


def send_serial(data: bytes):
    """Bezpečný zápis na sériový port (chráněno zámkem)."""
    global serial_conn
    with serial_lock:
        if serial_conn and serial_conn.is_open:
            serial_conn.write(data)
            print(f"Odesláno na serial: {data.hex()}")
        else:
            print("Serial není otevřen.")


# =========================
#   ŘÁDKOVÝ READER (10 Hz)
# =========================

def _reader_loop():
    """
    Low-latency čtení:
    - neblokující port (timeout=0)
    - čteme okamžitě, co je k dispozici (in_waiting)
    - jakmile dorazí '\\n', ihned voláme _line_handler(text)
    """
    global serial_conn, _line_handler
    buf = bytearray()

    while not _reader_stop.is_set():
        try:
            with serial_lock:
                s = serial_conn

            if not s or not s.is_open:
                time.sleep(0.02)
                continue

            n = s.in_waiting
            if n:
                chunk = s.read(n)  # vrátí hned, co je v ingress bufferu
                if chunk:
                    buf.extend(chunk)
                    # Zpracuj všechny kompletní řádky
                    while True:
                        nl_idx = buf.find(b'\n')
                        if nl_idx < 0:
                            break
                        raw = buf[:nl_idx]              # bez \n
                        del buf[:nl_idx+1]              # odstraň vč. \n
                        text = raw.decode('ascii', errors='ignore').strip('\r')
                        if text and _line_handler:
                            try:
                                _line_handler(text)
                            except Exception as e:
                                print(f"Chyba v on_line handleru: {e}")
            else:
                # malý spánek pro nízké CPU a ~1ms latenci
                time.sleep(0.001)

        except Exception as e:
            print(f"Serial reader chyba: {e}")
            time.sleep(0.01)


def start_serial_reader(on_line):
    """Spustí vlákno, které čte řádky ze sériovky a volá on_line(str). Idempotentní."""
    global _reader_thread, _line_handler
    _line_handler = on_line
    if _reader_thread and _reader_thread.is_alive():
        return
    _reader_stop.clear()
    _reader_thread = threading.Thread(target=_reader_loop, name="serial-reader", daemon=True)
    _reader_thread.start()
    print("📡 Serial reader spuštěn.")


def stop_serial_reader():
    """Požádá reader o ukončení; skončí do ~1 ms (neblokující read)."""
    global _reader_thread
    if _reader_thread and _reader_thread.is_alive():
        _reader_stop.set()
        print("📴 Serial reader stop požadavek.")
