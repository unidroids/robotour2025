# gnss_serial.py
import serial
import threading
import queue
import time
import sys
from typing import Optional, Tuple, Callable

UBX_SYNC_1 = 0xB5
UBX_SYNC_2 = 0x62

class GnssSerialIO:
    def __init__(self, device='/dev/gnss1', baudrate=115200, read_fifo_size=30, write_fifo_size=30):
        self.device = device
        self.baudrate = baudrate
        self._ser = None

        self._read_fifo = queue.Queue(maxsize=read_fifo_size)
        self._write_fifo = queue.Queue(maxsize=write_fifo_size)

        self._read_event = threading.Event()
        self._write_event = threading.Event()
        self._stop_event = threading.Event()

        self._writer_thread = threading.Thread(target=self._writer, daemon=True)
        self._reader_thread = threading.Thread(target=self._reader, daemon=True)

        self.on_ubx_message: Optional[Callable[[int, int, bytes], None]] = None  # callback, will be used by dispatcher

    def open(self):
        self._ser = serial.Serial(self.device, self.baudrate, timeout=0.02)
        self._stop_event.clear()
        self._writer_thread.start()
        self._reader_thread.start()

    def close(self):
        self._stop_event.set()
        self._write_event.set()
        self._read_event.set()
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass

    # ---------- Writer thread ----------
    def send_ubx(self, ubx_bytes: bytes):
        """Non-blocking push UBX message to writer FIFO."""
        try:
            self._write_fifo.put_nowait(ubx_bytes)
            self._write_event.set()
        except queue.Full:
            print("[GnssSerialIO] Write FIFO full – dropping message!", file=sys.stderr)

    def _writer(self):
        while not self._stop_event.is_set():
            self._write_event.wait(timeout=0.1)
            self._write_event.clear()
            while not self._write_fifo.empty():
                try:
                    data = self._write_fifo.get_nowait()
                    if self._ser and self._ser.writable():
                        self._ser.write(data)
                except Exception as e:
                    print(f"[GnssSerialIO] Writer error: {e}", file=sys.stderr)

    # ---------- Reader thread ----------
    def _reader(self):
        sync = 0
        ubx_header = bytearray()
        payload = bytearray()
        expected_length = 0

        while not self._stop_event.is_set():
            try:
                b = self._ser.read(1)
                if not b:
                    continue
                b = b[0]

                # State machine: sync[2] + header[4] + payload[length] + checksum[2]
                if sync == 0:
                    if b == UBX_SYNC_1:
                        sync = 1
                elif sync == 1:
                    if b == UBX_SYNC_2:
                        sync = 2
                        ubx_header = bytearray()
                    else:
                        sync = 0
                elif sync == 2:
                    ubx_header.append(b)
                    if len(ubx_header) == 4:
                        msg_class = ubx_header[0]
                        msg_id = ubx_header[1]
                        length = ubx_header[2] | (ubx_header[3] << 8)
                        expected_length = length
                        payload = bytearray()
                        sync = 3
                elif sync == 3:
                    payload.append(b)
                    if len(payload) == expected_length:
                        sync = 4
                        cksum = bytearray()
                elif sync == 4:
                    if len(cksum) < 2:
                        cksum.append(b)
                    if len(cksum) == 2:
                        # Validate checksum
                        if self._ubx_checksum(ubx_header + payload) == tuple(cksum):
                            # Valid UBX message
                            msg_class = ubx_header[0]
                            msg_id = ubx_header[1]
                            self._push_read_fifo(msg_class, msg_id, bytes(payload))
                        else:
                            print(f"[GnssSerialIO] Invalid UBX checksum! (class=0x{ubx_header[0]:02X}, id=0x{ubx_header[1]:02X})", file=sys.stderr)
                        sync = 0

            except Exception as e:
                print(f"[GnssSerialIO] Reader error: {e}", file=sys.stderr)

    def _ubx_checksum(self, data: bytes) -> Tuple[int, int]:
        ck_a = 0
        ck_b = 0
        for b in data:
            ck_a = (ck_a + b) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        return (ck_a, ck_b)

    def _push_read_fifo(self, msg_class: int, msg_id: int, payload: bytes):
        try:
            self._read_fifo.put_nowait((msg_class, msg_id, payload))
            self._read_event.set()
            if self.on_ubx_message:
                self.on_ubx_message(msg_class, msg_id, payload)
        except queue.Full:
            print("[GnssSerialIO] Read FIFO full – dropping UBX!", file=sys.stderr)

    # ---------- For dispatcher/handler integration ----------
    def get_ubx_message(self, timeout=None) -> Optional[Tuple[int, int, bytes]]:
        """Blocking read – for dispatcher: waits for new UBX (with timeout if set)."""
        try:
            return self._read_fifo.get(timeout=timeout)
        except queue.Empty:
            return None

    def wait_for_data(self, timeout=None):
        return self._read_event.wait(timeout=timeout)

    def clear_data_event(self):
        self._read_event.clear()

# ------------- DEMO -------------
if __name__ == '__main__':
    gnss = GnssSerialIO('/dev/gnss1')
    gnss.open()

    try:
        while True:
            # Blocking: wait for new UBX message in FIFO
            msg = gnss.get_ubx_message(timeout=1.0)
            if msg:
                msg_class, msg_id, payload = msg
                print(f"UBX {msg_class:02X} {msg_id:02X} {len(payload)} bytes")
            else:
                print(".", end="", flush=True)
    except KeyboardInterrupt:
        print("\nBye")
    finally:
        gnss.close()
