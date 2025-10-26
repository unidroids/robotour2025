"""hb_serial.py – nízkoúrovňové I/O pro hoverboard přes UART (Time‑Critical)

⚠️ DŮLEŽITÉ: Soubor je pojmenován **hb_serial.py** (ne `serial.py`),
aby se předešlo kolizi se závislostí **pyserial** (modul `serial`).

- Oddělená vlákna pro RX a TX
- RX: čte byty, předává do DriveParser.feed(), validované zprávy (DriveRx)
      ukládá do RX FIFO (queue.Queue)
- TX: vybírá z TX FIFO (bytes rámce) a posílá přes UART
- Bez blokujících printů v hot‑path, jen počitadla

Zařízení: '/dev/howerboard', 921600 Bd, 8N1

Závislosti: pyserial (importováno jako `import serial as pyserial`), parser.DriveParser
"""
from __future__ import annotations

import threading
import queue
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import serial as pyserial  # pyserial – vyhneme se kolizi názvů

from parser import DriveParser, DriveRx  # náš inkrementální parser

DEFAULT_DEVICE = "/dev/howerboard"
DEFAULT_BAUD = 921600

__all__ = [
    "SerialConfig",
    "HoverboardSerial",
]


@dataclass(slots=True)
class SerialConfig:
    device: str = DEFAULT_DEVICE
    baudrate: int = DEFAULT_BAUD
    timeout_s: float = 0.02       # read timeout (non‑blocking-ish)
    write_timeout_s: float = 0.1  # write timeout
    rx_fifo_size: int = 256
    tx_fifo_size: int = 256
    read_chunk: int = 4096        # kolik max číst naráz
    reconnect_delay_s: float = 0.5


class HoverboardSerial:
    """UART I/O s RX/TX vlákny a FIFO frontami."""

    def __init__(self, cfg: Optional[SerialConfig] = None):
        self.cfg = cfg or SerialConfig()
        self._ser: Optional[pyserial.Serial] = None

        self._rx_fifo: "queue.Queue[DriveRx]" = queue.Queue(maxsize=self.cfg.rx_fifo_size)
        self._tx_fifo: "queue.Queue[bytes]" = queue.Queue(maxsize=self.cfg.tx_fifo_size)

        self._stop_event = threading.Event()
        self._writer_wakeup = threading.Event()
        self._reader_thr = threading.Thread(target=self._reader_loop, name="drive-rx", daemon=True)
        self._writer_thr = threading.Thread(target=self._writer_loop, name="drive-tx", daemon=True)

        self._parser = DriveParser(max_line=128)
        # počitadla
        self._rx_bytes = 0
        self._tx_bytes = 0
        self._rx_msgs = 0
        self._tx_frames = 0
        self._rx_overflows = 0
        self._tx_overflows = 0
        self._open_failures = 0

        self._opened_lock = threading.Lock()
        self._opened = False

    # ---------------- lifecycle ----------------
    def start(self) -> None:
        with self._opened_lock:
            if self._opened:
                return
            self._stop_event.clear()
            self._open_port()
            self._writer_thr.start()
            self._reader_thr.start()
            self._opened = True

    def stop(self) -> None:
        with self._opened_lock:
            if not self._opened:
                return
            self._stop_event.set()
            self._writer_wakeup.set()
            try:
                if self._reader_thr.is_alive():
                    self._reader_thr.join(timeout=0.3)
            except Exception:
                pass
            try:
                if self._writer_thr.is_alive():
                    self._writer_thr.join(timeout=0.3)
            except Exception:
                pass
            try:
                if self._ser:
                    self._ser.close()
            except Exception:
                pass
            self._ser = None
            self._opened = False

    # ---------------- public TX/RX API ----------------
    def send_frame(self, frame: bytes) -> bool:
        if not frame:
            return False
        try:
            self._tx_fifo.put_nowait(frame)
            self._writer_wakeup.set()
            return True
        except queue.Full:
            self._tx_overflows += 1
            return False

    def get_rx(self, timeout: Optional[float] = None) -> Optional[DriveRx]:
        try:
            return self._rx_fifo.get(timeout=timeout)
        except queue.Empty:
            return None

    @property
    def rx_queue(self) -> "queue.Queue[DriveRx]":
        return self._rx_fifo

    def stats(self) -> Tuple[int, int, int, int, int, int, int, int, int]:
        return (
            self._rx_bytes,
            self._tx_bytes,
            self._rx_msgs,
            self._tx_frames,
            self._rx_overflows,
            self._tx_overflows,
            self._parser.bad_lines,
            self._parser.too_long_lines,
            self._parser.unknown_codes,
        )

    # ---------------- internals ----------------
    def _open_port(self) -> None:
        cfg = self.cfg
        try:
            self._ser = pyserial.Serial(
                cfg.device,
                cfg.baudrate,
                timeout=cfg.timeout_s,
                write_timeout=cfg.write_timeout_s,
            )
            try:
                self._ser.exclusive = True  # type: ignore[attr-defined]
            except Exception:
                pass
        except Exception:
            self._open_failures += 1
            self._ser = None

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            if not self._ser or not self._ser.is_open:
                self._open_port()
                if not self._ser:
                    time.sleep(self.cfg.reconnect_delay_s)
                    continue

            try:
                data = self._ser.read(self.cfg.read_chunk)
                if data:
                    self._rx_bytes += len(data)
                    msgs = self._parser.feed(data)
                    for m in msgs:
                        try:
                            self._rx_fifo.put_nowait(m)
                            self._rx_msgs += 1
                        except queue.Full:
                            self._rx_overflows += 1
                else:
                    time.sleep(0.001)
            except Exception:
                try:
                    if self._ser:
                        self._ser.close()
                except Exception:
                    pass
                self._ser = None
                time.sleep(self.cfg.reconnect_delay_s)

    def _writer_loop(self) -> None:
        while not self._stop_event.is_set():
            fired = self._writer_wakeup.wait(timeout=0.05)
            if fired:
                self._writer_wakeup.clear()

            if not self._ser or not self._ser.is_open:
                time.sleep(0.01)
                continue

            try:
                while True:
                    try:
                        frame = self._tx_fifo.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        n = self._ser.write(frame)
                        self._tx_bytes += int(n or 0)
                        self._tx_frames += 1
                    except Exception:
                        try:
                            if self._ser:
                                self._ser.close()
                        except Exception:
                            pass
                        self._ser = None
                        break
            except Exception:
                time.sleep(0.01)


if __name__ == "__main__":
    hb = HoverboardSerial()
    hb.start()
    print("HoverboardSerial started. Press Ctrl+C to stop.")
    try:
        t0 = time.time()
        while True:
            m = hb.get_rx(timeout=0.05)
            if m is not None:
                print("RX:", m)
            if time.time() - t0 > 1.0:
                t0 = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        print("Stopping...")
        hb.stop()
        print("Stopped. Stats:", hb.stats())
