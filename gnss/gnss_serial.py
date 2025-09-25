# gnss_serial.py
import serial
import threading
import queue
import sys
from typing import Optional, Tuple

from parse_gnss_feed import GnssStreamParser, GnssParseResult

class GnssSerialIO:
    """
    GNSS I/O s oddělenými výstupními frontami pro UBX a NMEA.
    Reader: byte-by-byte přes GnssStreamParser(max_ubx_payload=512), bez parsování frame/sentence.
    """

    def __init__(self,
                 device: str = '/dev/gnss1',
                 baudrate: int = 921600,
                 ubx_fifo_size: int = 60,
                 nmea_fifo_size: int = 60,
                 write_fifo_size: int = 60,
                 max_ubx_payload: int = 512):
        self.device = device
        self.baudrate = baudrate
        self._ser: Optional[serial.Serial] = None

        self._ubx_fifo: "queue.Queue[bytes]" = queue.Queue(maxsize=ubx_fifo_size)
        self._nmea_fifo: "queue.Queue[bytes]" = queue.Queue(maxsize=nmea_fifo_size)
        self._write_fifo: "queue.Queue[bytes]" = queue.Queue(maxsize=write_fifo_size)

        self._stop_event = threading.Event()
        self._write_event = threading.Event()
        self._reader_thread = threading.Thread(target=self._reader, daemon=True)
        self._writer_thread = threading.Thread(target=self._writer, daemon=True)

        self._parser = GnssStreamParser(max_ubx_payload=max_ubx_payload, junk_flush_len=64)
        self._err_corrupted = 0
        self._err_checksum = 0

    # ---------- lifecycle ----------
    def open(self):
        self._ser = serial.Serial(self.device, self.baudrate, timeout=0.02)
        self._stop_event.clear()
        self._writer_thread.start()
        self._reader_thread.start()

    def close(self):
        self._stop_event.set()
        self._write_event.set()
        try:
            if self._reader_thread.is_alive():
                self._reader_thread.join(timeout=0.2)
        except Exception:
            pass
        try:
            if self._writer_thread.is_alive():
                self._writer_thread.join(timeout=0.2)
        except Exception:
            pass
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass

    # ---------- writer ----------
    def send_ubx(self, ubx_bytes: bytes) -> bool:
        try:
            self._write_fifo.put_nowait(ubx_bytes)
            self._write_event.set()
            return True
        except queue.Full:
            print("[GnssSerialIO] Write FIFO full – dropping UBX message!", file=sys.stderr)
            return False

    def _writer(self):
        while not self._stop_event.is_set():
            self._write_event.wait(timeout=0.1)
            self._write_event.clear()
            while not self._write_fifo.empty():
                try:
                    data = self._write_fifo.get_nowait()
                except queue.Empty:
                    break
                try:
                    if self._ser and self._ser.writable():
                        self._ser.write(data)
                except Exception as e:
                    print(f"[GnssSerialIO] Writer error: {e}", file=sys.stderr)

    # ---------- public getters (blocking pop) ----------
    def get_ubx_frame(self, timeout: Optional[float] = None) -> Optional[bytes]:
        """Vrátí celý UBX rámec (B5 62 ... ckA ckB)."""
        try:
            return self._ubx_fifo.get(timeout=timeout)
        except queue.Empty:
            return None

    def get_nmea_sentence(self, timeout: Optional[float] = None) -> Optional[bytes]:
        """Vrátí celou NMEA větu (včetně CRLF)."""
        try:
            return self._nmea_fifo.get(timeout=timeout)
        except queue.Empty:
            return None

    def get_error_counters(self) -> Tuple[int, int]:
        return (self._err_corrupted, self._err_checksum)

    # ---------- legacy convenience (parsing in reader) ----------
    def get_ubx_message(self, timeout: Optional[float] = None) -> Optional[Tuple[int, int, bytes]]:
        """
        (Legacy) Vytáhne frame a rozparsuje na (class,id,payload).
        Používej spíše get_ubx_frame() + dispatcher v separátním vlákně.
        """
        frm = self.get_ubx_frame(timeout=timeout)
        if not frm:
            return None
        if len(frm) < 8 or frm[0] != 0xB5 or frm[1] != 0x62:
            return None
        msg_class = frm[2]
        msg_id = frm[3]
        length = frm[4] | (frm[5] << 8)
        payload = frm[6:6+length]
        return (msg_class, msg_id, payload)

    # ---------- reader thread ----------
    def _reader(self):
        while not self._stop_event.is_set():
            try:
                b = self._ser.read(1)
                if not b:
                    continue
                typ, msg = self._parser.feed(b[0])

                if typ == GnssParseResult.PROCESSING:
                    continue

                if typ == GnssParseResult.NMEA:
                    try:
                        self._nmea_fifo.put_nowait(msg)   # raw bytes, včetně CRLF
                    except queue.Full:
                        # nejdeme blokovat reader; jen počítáme chybu
                        self._err_corrupted += 1
                        print("[GnssSerialIO] NMEA FIFO full – dropping sentence!", file=sys.stderr)

                elif typ == GnssParseResult.UBX:
                    try:
                        self._ubx_fifo.put_nowait(msg)    # raw frame bytes
                    except queue.Full:
                        self._err_corrupted += 1
                        print("[GnssSerialIO] UBX FIFO full – dropping frame!", file=sys.stderr)

                elif typ == GnssParseResult.CHECKSUM_ERROR:
                    self._err_checksum += 1
                    # jen lehký log (bez dlouhých dumpů)
                    print("[GnssSerialIO] Checksum error.", file=sys.stderr)

                elif typ == GnssParseResult.CORRUPTED:
                    self._err_corrupted += 1
                    # volitelné: krátký log
                    # print(f"[GnssSerialIO] Corrupted block (len={len(msg)})", file=sys.stderr)

            except Exception as e:
                print(f"[GnssSerialIO] Reader error: {e}", file=sys.stderr)

# --------- DEMO (nepovinné) ---------
if __name__ == '__main__':
    import time
    gnss = GnssSerialIO('/dev/gnss1', baudrate=921600)
    gnss.open()
    print("GNSS opened. Ctrl+C to stop.")
    try:
        while True:
            if (frm := gnss.get_ubx_frame(timeout=0.01)):
                mc, mi, ln = frm[2], frm[3], (frm[4] | (frm[5] << 8))
                print(f"UBX {mc:02X} {mi:02X} ({ln} B)")
            if (nmea := gnss.get_nmea_sentence(timeout=0.0)):
                print("NMEA:", nmea.decode(errors='ignore').strip())
            time.sleep(0.005)
    except KeyboardInterrupt:
        c1, c2 = gnss.get_error_counters()
        print(f"\nBye. corrupted={c1}, checksum_errors={c2}")
    finally:
        gnss.close()
        print("GNSS closed.")
