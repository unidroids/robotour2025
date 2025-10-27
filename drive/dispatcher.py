"""dispatcher.py – jednovláknový router příchozích zpráv na handlery

Běží ve vlastním vlákně, čte z RX FIFO (HoverboardSerial.rx_queue) a podle
kódu zprávy (IAM/INM/MSM/ODM/DIM/SEM/SWM) volá zaregistrované handlery.

Design cíle:
  - Time‑critical: minimální režie, žádné I/O v hot‑path
  - Bezpečnost: výjimka v handleru nezabije vlákno, jen se zapíše do počitadel
  - Rozšiřitelnost: jednoduché API pro registraci handlerů + default handler

Použití (z service.py):
  disp = MessageDispatcher(hb_serial)
  disp.register_handler('IAM', handlers.ack.handle)
  disp.register_handler('INM', handlers.nack.handle)
  ...
  disp.start()

Handler je libovolný callable s podpisem: handle(msg: DriveRx) -> None

Pozn.: Tohle je "router" – vlastní logika je v modulech handlers/*, které
můžou logovat pomocí print (zatím), zapisovat do souborů atd.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, Optional

#from parser import DriveRx, VALID_CODES
from hb_serial import HoverboardSerial  # náš modul serial.py (pyserial je importován uvnitř něj)

__all__ = [
    "DispatcherConfig",
    "MessageDispatcher",
]


@dataclass(slots=True)
class DispatcherConfig:
    poll_timeout_s: float = 0.02   # čekání na zprávu z RX FIFO
    max_batch: int = 64            # kolik zpráv max. zpracovat na jeden průchod
    idle_sleep_s: float = 0.001    # mikrospánek, když není co dělat
    debug: bool = False


@dataclass(slots=True)
class DispatcherStats:
    started_at: float = field(default_factory=time.monotonic)
    processed: int = 0
    unhandled: int = 0
    handler_errors: int = 0
    per_code: Dict[str, int] = field(default_factory=dict)
    last_error: Optional[str] = None


class MessageDispatcher:
    """Čte DriveRx z hb_serial.rx_queue a volá příslušné handlery."""

    def __init__(self, hb_serial: HoverboardSerial, cfg: Optional[DispatcherConfig] = None):
        self._ser = hb_serial
        self._cfg = cfg or DispatcherConfig()
        self._stop = threading.Event()
        self._thr = threading.Thread(target=self._loop, name="drive-dispatch", daemon=True)
        self._handlers: Dict[str, Callable[[DriveRx], None]] = {}
        self._default_handler: Optional[Callable[[DriveRx], None]] = None
        self._stats = DispatcherStats()
        self._lock = threading.Lock()   # chrání registry handlerů a stats

    # ---------- Lifecycle ----------
    def start(self) -> None:
        if self._thr.is_alive():
            return
        self._stop.clear()
        self._thr.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thr.is_alive():
            try:
                self._thr.join(timeout=0.3)
            except Exception:
                pass

    # ---------- Handlery ----------
    def register_handler(self, code: str, handler: Callable[[DriveRx], None]) -> None:
        code = code.upper()
        if code not in VALID_CODES:
            raise ValueError(f"Unknown code '{code}' – not in VALID_CODES: {sorted(VALID_CODES)}")
        with self._lock:
            self._handlers[code] = handler

    def set_default_handler(self, handler: Optional[Callable[[DriveRx], None]]) -> None:
        with self._lock:
            self._default_handler = handler

    # ---------- Stats/Introspection ----------
    def stats(self) -> DispatcherStats:
        with self._lock:
            # vrací shallow copy (per_code by reference – ok pro rychlost; neměnit zvenku!)
            return self._stats

    # ---------- Main loop ----------
    def _loop(self) -> None:
        cfg = self._cfg
        rxq = self._ser.rx_queue
        get = rxq.get

        while not self._stop.is_set():
            processed_now = 0
            try:
                # blokuj (krátce) na první zprávu, ať nespotřebováváme CPU
                msg: Optional[DriveRx] = None
                try:
                    msg = get(timeout=cfg.poll_timeout_s)
                except Exception:
                    msg = None

                if msg is None:
                    time.sleep(cfg.idle_sleep_s)
                    continue

                # zpracuj první a pak neblokuj, dokud nevyčerpáme batch
                self._dispatch_one(msg)
                processed_now += 1

                while processed_now < cfg.max_batch:
                    try:
                        msg = get_nowait(rxq)
                    except _Empty:
                        break
                    self._dispatch_one(msg)
                    processed_now += 1

            except Exception as e:
                with self._lock:
                    self._stats.last_error = f"loop: {e!r}"
                # ochranná pauza
                time.sleep(0.001)

    # malé optimalizace pro lokální lookups
    def _dispatch_one(self, msg: DriveRx) -> None:
        handler: Optional[Callable[[DriveRx], None]]
        with self._lock:
            handler = self._handlers.get(msg.code, self._default_handler)
        if handler is None:
            # nezaregistrovaný handler
            with self._lock:
                self._stats.unhandled += 1
                self._stats.per_code[msg.code] = self._stats.per_code.get(msg.code, 0) + 1
                self._stats.processed += 1
            return

        try:
            handler(msg)
            with self._lock:
                self._stats.per_code[msg.code] = self._stats.per_code.get(msg.code, 0) + 1
                self._stats.processed += 1
        except Exception as e:
            with self._lock:
                self._stats.handler_errors += 1
                self._stats.last_error = f"handler[{msg.code}]: {e!r}"


# ---- drobné utility pro non‑blocking get ----
import queue as _q

_Empty = _q.Empty

def get_nowait(q: _q.Queue):
    return q.get_nowait()


# ---- jednoduchý self‑test ----
if __name__ == "__main__":
    # Minimal self‑test bez reálného UARTu: vytvoříme fake serial a naplníme RX zprávami.
    from collections import deque

    class _FakeSerial:
        def __init__(self):
            self._q: _q.Queue[DriveRx] = _q.Queue()
        @property
        def rx_queue(self):
            return self._q

    def _print_handler(msg: DriveRx) -> None:
        print(f"HANDLER {msg.code}: {msg.values}")

    fake = _FakeSerial()
    disp = MessageDispatcher(fake, DispatcherConfig(debug=True))
    disp.register_handler('IAM', _print_handler)
    disp.register_handler('INM', _print_handler)
    disp.set_default_handler(_print_handler)
    disp.start()

    # Vložíme pár zpráv
    t = time.monotonic()
    for code in ("IAM", "INM", "MSM"):
        fake.rx_queue.put(DriveRx(code=code, values=(1, 2, 3, 4), raw=b"$...\r\n", t_mono=t))

    time.sleep(0.1)
    print("stats:", disp.stats())
    disp.stop()
