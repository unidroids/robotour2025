# drive/ack_rtt_tester.py
from __future__ import annotations
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple, Callable

# --- rámcové konstanty protokolu ---
STX = 251
MTX = 252
ETX = 253

def _assert_param(v: int) -> None:
    iv = int(v)
    if iv < 0 or iv > 250:
        raise ValueError(f"Param {iv} out of range 0..250")

def build_frame(cmd: int, p1: int, p2: int, p3: int, p4: int) -> bytes:
    """Sestaví binární rámec dle specifikace."""
    for v in (cmd, p1, p2, p3, p4):
        _assert_param(v)
    return bytes([STX, cmd, p1, p2, p3, p4, MTX, cmd, p1, p2, p3, p4, ETX])

# --- base-251 timestamp packing (všechny parametry 0..250) ---

def base251_encode(u32: int) -> Tuple[int, int, int, int]:
    """
    Zakóduje 0..(251^4 - 1) do 4 'číslic' 0..250.
    Hodí se pro microseconds modulo 251^4 (~3.97 s okno).
    """
    d1 = u32 % 251
    u32 //= 251
    d2 = u32 % 251
    u32 //= 251
    d3 = u32 % 251
    u32 //= 251
    d4 = u32 % 251
    return d1, d2, d3, d4

def base251_decode(d1: int, d2: int, d3: int, d4: int) -> int:
    return (((d4 * 251) + d3) * 251 + d2) * 251 + d1

# --- ACK/NACK stav ---

@dataclass(slots=True)
class AckResult:
    ok: bool                 # True=ACK, False=NACK
    cmd: int
    p: Tuple[int, int, int, int]
    rtt_ms: float            # měřený round-trip time v ms
    nack_reason: Optional[str] = None  # jen při NACK

class AckNackMatcher:
    """
    Matching podle (cmd,p1..p4) s jedním outstanding dotazem.
    - pace_interval_ms: minimální rozestup mezi odeslanými rámci (anti-collision)
    - resend_on_nack_quality: povolit resend, pokud NACK důvod === 'QUALITY'
    """
    def __init__(self, send_func: Callable[[bytes], bool],
                 pace_interval_ms: float = 10.0,
                 timeout_ms: float = 20.0,
                 max_retries: int = 2,
                 resend_on_nack_quality: bool = True) -> None:
        self.send_func = send_func
        self.pace_interval_ms = pace_interval_ms
        self.timeout_ms = timeout_ms
        self.max_retries = max_retries
        self.resend_on_nack_quality = resend_on_nack_quality

        self._lock = threading.Lock()
        self._wait_ev = threading.Event()
        self._pending: Optional[Tuple[int, Tuple[int, int, int, int], float]] = None  # (cmd,(p1..p4), t_send_mono)
        self._last_send_mono: float = 0.0
        self._result: Optional[AckResult] = None

    def _pace(self) -> None:
        """Zajistí min. rozestup mezi odesláními."""
        now = time.monotonic()
        delta_ms = (now - self._last_send_mono) * 1000.0
        wait_ms = self.pace_interval_ms - delta_ms
        if wait_ms > 0:
            time.sleep(wait_ms / 1000.0)

    def send_and_wait(self, cmd: int, p1: int, p2: int, p3: int, p4: int) -> AckResult:
        """
        Stop-and-wait:
        - pošli rámec
        - čekej na ACK/NACK (timeout)
        - při NACK s důvodem 'QUALITY' může opakovat (pokud povoleno)
        - při timeoutu opakuje do max_retries
        """
        attempt = 0
        reason: Optional[str] = None

        while True:
            attempt += 1
            self._pace()

            frame = build_frame(cmd, p1, p2, p3, p4)
            t_send = time.monotonic()
            with self._lock:
                self._result = None
                self._wait_ev.clear()
                self._pending = (cmd, (p1, p2, p3, p4), t_send)

            ok = self.send_func(frame)
            self._last_send_mono = t_send
            if not ok:
                # odeslání selhalo na transportu
                with self._lock:
                    self._pending = None
                return AckResult(False, cmd, (p1, p2, p3, p4), 0.0, nack_reason="TX_ERROR")

            # čekej na ACK/NACK
            got = self._wait_ev.wait(self.timeout_ms / 1000.0)

            with self._lock:
                res = self._result
                self._pending = None

            if not got or res is None:
                # timeout
                if attempt <= self.max_retries:
                    continue
                return AckResult(False, cmd, (p1, p2, p3, p4), 0.0, nack_reason="TIMEOUT")

            if res.ok:
                return res  # ACK

            # NACK
            reason = res.nack_reason or "NACK"
            if reason.upper() == "QUALITY" and self.resend_on_nack_quality and attempt <= self.max_retries:
                # rychlý resend po pace
                continue
            return res

    # Tyto dvě metody zavolej z RX dispečeru (viz MessageDispatcher handlery):
    def on_ack(self, cmd: int, p1: int, p2: int, p3: int, p4: int) -> None:
        with self._lock:
            if self._pending is None:
                return
            pcmd, pp, t_send = self._pending
            if pcmd == cmd and pp == (p1, p2, p3, p4):
                rtt_ms = (time.monotonic() - t_send) * 1000.0
                self._result = AckResult(True, cmd, pp, rtt_ms, None)
                self._wait_ev.set()

    def on_nack(self, cmd: int, p1: int, p2: int, p3: int, p4: int, reason: Optional[str]) -> None:
        with self._lock:
            if self._pending is None:
                return
            pcmd, pp, t_send = self._pending
            if pcmd == cmd and pp == (p1, p2, p3, p4):
                rtt_ms = (time.monotonic() - t_send) * 1000.0
                self._result = AckResult(False, cmd, pp, rtt_ms, reason or "NACK")
                self._wait_ev.set()

# --- „probe“: měření RTT přes cmd=50 a timestamp v p1..p4 ---

def make_probe_params_from_now() -> Tuple[int, int, int, int]:
    """
    P1..P4 = base-251 zakódovaný timestamp (microseconds mod 251^4).
    """
    t_us = int(time.monotonic() * 1_000_000)
    t_mod = t_us % (251**4)
    return base251_encode(t_mod)

def probe_decode_age_ms(p1: int, p2: int, p3: int, p4: int) -> float:
    """
    Volitelná diagnostika: pokud FW vkládá do ACK i vlastní timestamp, lze porovnat.
    Tady pouze ukázka dekódování param z p1..p4 do u32.
    """
    t_mod = base251_decode(p1, p2, p3, p4)
    # bez znalosti absolutního času FW je to čistě referenční hodnota
    return float(t_mod) / 1000.0  # „ms“ v rámci modulo prostoru

# --- Jednoduchý testovací běh: perioda 55 ms, print P50/P95/P99 ---

class RttStats:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.values_ms: list[float] = []

    def add(self, v_ms: float) -> None:
        with self._lock:
            self.values_ms.append(v_ms)

    def snapshot(self) -> Tuple[float, float, float, float]:
        with self._lock:
            data = sorted(self.values_ms)
        if not data:
            return (0.0, 0.0, 0.0, 0.0)
        def q(p: float) -> float:
            if not data:
                return 0.0
            idx = int((len(data)-1) * p)
            return data[idx]
        return (data[0], q(0.5), q(0.95), q(0.99))

# --- Integrace s existujícím hb_serial/MessageDispatcher ---

if __name__ == '__main__':
    # Předpoklad:
    # - hb_serial.HoverboardSerial: .start(), .stop(), .send_frame(bytes)->bool
    # - MessageDispatcher: .register_handler(code:str, handler), .start(), .stop()
    #
    # ACK zprávy: kód 'ACK' -> payload nese "cmd,p1,p2,p3,p4" (ASCII nebo binárně – zde demonstrujeme genericky)
    # NACK zprávy: kód 'INM' (Input Nack Message) -> payload nese totéž + 'reason'
    #
    from hb_serial import HoverboardSerial
    from dispatcher import MessageDispatcher  # přizpůsob dle tvého projektu

    import sys
    import traceback

    hb = HoverboardSerial()
    hb.start()

    md = MessageDispatcher(hb)

    # --- Ack matcher s pacingem 10 ms, timeout 20 ms, retries 2 ---
    matcher = AckNackMatcher(
        send_func=hb.send_frame,
        pace_interval_ms=10.0,
        timeout_ms=20.0,
        max_retries=2,
        resend_on_nack_quality=True
    )

    # --- RTT statistiky ---
    stats = RttStats()

    # --- Handlery z RX dispečeru (přizpůsob parsování podle reálného payloadu) ---
    class AckHandler:
        def handle(self, message_bytes: bytes):
            """
            Očekáváme, že FW do ACK zkopíruje původní cmd a p1..p4.
            Pokud jsou to ASCII csv (např. "ACK,50,1,2,3,4"), parsuj takto.
            Pokud binárně, uprav dekódování zde.
            """
            try:
                text = message_bytes.decode(errors='ignore').strip()
                # příklady: "ACK,50,1,2,3,4" nebo jen "
