"""hb_serial.py – nízkoúrovňové I/O pro hoverboard přes UART (Time‑Critical)

⚠️ DŮLEŽITÉ: Soubor je pojmenován **hb_serial.py** (ne `serial.py`),
aby se předešlo kolizi se závislostí **pyserial** (modul `serial`).

- Oddělená vlákna pro RX a TX
- RX: blokuje na 1 bajt s timeoutem; po přijetí vyčerpá `in_waiting` (nízká latence)
- TX: vybírá z TX FIFO (bytes rámce) a posílá přes UART
- Datalogger: zapisuje odeslané rámce (HEX) a přijaté ASCII řádky (oddělené CRLF)
- Bez zbytečných `sleep()` v hot‑path; blokování řídí `read(1)` s konfigurovatelným timeoutem

Zařízení: '/dev/howerboard', 921600 Bd, 8N1

Závislosti: pyserial (importováno jako `import serial as pyserial`), parser.DriveParser
"""
from __future__ import annotations

import threading
import queue
import time
import os
from dataclasses import dataclass
from typing import Optional, Tuple

import serial as pyserial  # pyserial – vyhneme se kolizi názvů

from parser import DriveParser, DriveRx  # náš inkrementální parser

DEFAULT_DEVICE = "/dev/howerboard"
DEFAULT_BAUD = 921600
DEFAULT_LOG_DIR = "/data/robot/drive"

__all__ = [
    "SerialConfig",
    "HoverboardSerial",
]


@dataclass(slots=True)
class SerialConfig:
    device: str = DEFAULT_DEVICE
    baudrate: int = DEFAULT_BAUD
    timeout_s: float = 1.0        # read(1) timeout; max doba blokování mezi kontrolami stop_event
    write_timeout_s: float = 0.1  # write timeout
    rx_fifo_size: int = 256
    tx_fifo_size: int = 256
    read_chunk: int = 4096        # kolik max číst naráz
    reconnect_delay_s: float = 0.5
    # Datalogger
    enable_log: bool = True
    log_dir: str = DEFAULT_LOG_DIR
    log_flush: bool = False       # True => flush po každé řádce (vyšší režie)


class HoverboardSerial:
    """UART I/O s RX/TX vlákny a FIFO frontami + datalogger."""

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

        # Datalogger
        self._rx_linebuf = bytearray()
        self._log_tx_fh: Optional[object] = None
        self._log_rx_fh: Optional[object] = None
        self._log_tx_path: Optional[str] = None
        self._log_rx_path: Optional[str] = None

    # ---------------- lifecycle ----------------
    def start(self) -> None:
        with self._opened_lock:
            if self._opened:
                return
            self._stop_event.clear()
            self._prepare_logger()
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
                    self._reader_thr.join(timeout=self.cfg.timeout_s + 0.2)
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
            self._close_logger()
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

    # ---- Logger helpers ----
    def _prepare_logger(self) -> None:
        if not self.cfg.enable_log:
            return
        # Vytvoř adresář dle data: <log_dir>/YYYY-MM-DD/
        try:
            day_dir = time.strftime("%Y-%m-%d", time.localtime())
            base_dir = os.path.join(self.cfg.log_dir, day_dir)
            os.makedirs(base_dir, exist_ok=True)
        except Exception:
            # Nelze vytvořit adresář -> vypnout logging
            self.cfg.enable_log = False
            return
        # Jméno souboru zachovává původní formát s datem i časem kvůli konzistenci
        ts = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
        self._log_tx_path = os.path.join(base_dir, f"{ts}-tx.dat")
        self._log_rx_path = os.path.join(base_dir, f"{ts}-rx.dat")
        try:
            self._log_tx_fh = open(self._log_tx_path, "w", encoding="ascii", buffering=1)
            self._log_rx_fh = open(self._log_rx_path, "w", encoding="ascii", buffering=1)
        except Exception:
            # Nepodařilo se otevřít soubory -> vypnout logging
            self._log_tx_fh = None
            self._log_rx_fh = None
            self.cfg.enable_log = False

    def _close_logger(self) -> None:
        for fh in (self._log_tx_fh, self._log_rx_fh):
            try:
                if fh:
                    fh.flush()
                    fh.close()
            except Exception:
                pass
        self._log_tx_fh = None
        self._log_rx_fh = None

    def _log_tx(self, frame: bytes) -> None:
        if not (self.cfg.enable_log and self._log_tx_fh):
            return
        t = time.monotonic()
        try:
            self._log_tx_fh.write(f"{t:.6f} {frame.hex().upper()}\n")
            if self.cfg.log_flush:
                self._log_tx_fh.flush()
        except Exception:
            pass

    def _rx_log_feed(self, data: bytes) -> None:
        """Přijímaná data (ASCII). Při každém CRLF zapíše řádku s monotonic timestampem."""
        if not (self.cfg.enable_log and self._log_rx_fh):
            return
        self._rx_linebuf.extend(data)
        while True:
            i = self._rx_linebuf.find(b"\n")
            if i < 0:
                break
            line = bytes(self._rx_linebuf[:i])  # bez CRLF
            del self._rx_linebuf[: i + 2]
            t = time.monotonic()
            try:
                txt = line.decode("ascii", errors="replace")
                self._log_rx_fh.write(f"{t:.6f} {txt}\n")
                if self.cfg.log_flush:
                    self._log_rx_fh.flush()
            except Exception:
                pass

    # ---- RX/TX threads ----
    def _reader_loop(self) -> None:
        cfg = self.cfg
        while not self._stop_event.is_set():
            if not self._ser or not self._ser.is_open:
                self._open_port()
                if not self._ser:
                    time.sleep(cfg.reconnect_delay_s)
                    continue

            try:
                # 1) Okamžitě vyčti vše, co je dostupné bez blokace
                avail = self._ser.in_waiting
                if avail:
                    data = self._ser.read(min(avail, cfg.read_chunk))
                else:
                    # 2) Nemáme nic – blokuj na 1 bajt do timeoutu
                    data = self._ser.read(1)
                    # Pokud přišel aspoň jeden bajt, dočti zbytek, co stihl dorazit
                    if data:
                        avail = self._ser.in_waiting
                        if avail:
                            data += self._ser.read(min(avail, cfg.read_chunk))

                if data:
                    self._rx_bytes += len(data)
                    self._rx_log_feed(data)
                    msgs = self._parser.feed(data)
                    for m in msgs:
                        try:
                            self._rx_fifo.put_nowait(m)
                            self._rx_msgs += 1
                        except queue.Full:
                            self._rx_overflows += 1
                # žádné "sleep"; řízení tempa zajišťuje blokace read(1) timeoutem

            except Exception:
                try:
                    if self._ser:
                        self._ser.close()
                except Exception:
                    pass
                self._ser = None
                time.sleep(cfg.reconnect_delay_s)

    def _writer_loop(self) -> None:
        cfg = self.cfg
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
                        self._log_tx(frame)
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
