# drive/ack_nack.py
from __future__ import annotations
import time
import threading
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

# --- rámcové konstanty (podle tvé specifikace) ---
STX = 251
MTX = 252
ETX = 253

# --- typy ---
CmdKey = Tuple[int, int, int, int, int]  # (cmd, p1, p2, p3, p4)

@dataclass
class AckResult:
    ok: bool                    # True=ACK, False=NACK/timeout
    is_timeout: bool
    input_err: int = 0          # INM: quality/input error
    cmd_err: int = 0            # INM: command/param error
    rtt_ms: float = 0.0
    sent_mono_ns: int = 0
    ack_mono_ns: int = 0
    retries_done: int = 0

# --- util: validace rozsahu paramů 0..250 ---
def _assert_param(v: int) -> None:
    iv = int(v)
    if iv < 0 or iv > 250:
        raise ValueError(f"Param {iv} out of range 0..250")

# --- util: sestavení binárního rámce pro TX (tvůj formát) ---
def build_frame(cmd: int, p1: int, p2: int, p3: int, p4: int) -> bytes:
    for v in (cmd, p1, p2, p3, p4):
        _assert_param(v)
    # [STX, cmd, p1, p2, p3, p4, MTX, cmd, p1, p2, p3, p4, ETX]
    return bytes([STX, cmd, p1, p2, p3, p4, MTX, cmd, p1, p2, p3, p4, ETX])

# --- util: XOR8 checksum pro NMEA-like řetězce ---
def nmea_checksum(body: str) -> int:
    cs = 0
    for ch in body:
        cs ^= ord(ch)
    return cs & 0xFF

# --- util: parse celé NMEA-like věty b"$INM50,...*CS\r\n" -> ("INM50",[fieldy]) ---
def parse_nmea_line(msg: bytes) -> Tuple[str, list]:
    """
    Vrací (code_with_cmd, fields_as_strings).
    Vyhazuje ValueError, pokud chybí $, * nebo nesedí CS.
    """
    s = msg.decode(errors="ignore").strip()
    if not s.startswith("$"):
        raise ValueError("No $ at start")
    if "*" not in s:
        raise ValueError("No * (checksum separator)")
    body, cs_part = s[1:].split("*", 1)  # bez "$"
    cs_hex = cs_part.strip()[:2]
    try:
        cs_val = int(cs_hex, 16)
    except Exception as e:
        raise ValueError(f"Bad CS hex: {e}")
    calc = nmea_checksum(body)
    if calc != cs_val:
        raise ValueError(f"Checksum mismatch: got {cs_val:02X}, calc {calc:02X}")

    # body je "INM50,..." nebo "IAM50,..."
    if "," in body:
        code_with_cmd, rest = body.split(",", 1)
        fields = rest.split(",") if rest else []
    else:
        code_with_cmd = body
        fields = []
    return code_with_cmd, fields

# --- util: rozdělení "INM50" -> ("INM", 50) ---
def split_code_and_cmd(code_with_cmd: str) -> Tuple[str, Optional[int]]:
    base = code_with_cmd[:3].upper()
    tail = code_with_cmd[3:]
    if not tail:
        return base, None
    try:
        return base, int(tail)
    except ValueError:
        return base, None

# --- base-251 kódování/decodování 32bit čísla do p1..p4 (0..250) ---
BASE = 251

def base251_encode_u32(val: int) -> Tuple[int, int, int, int]:
    v = val % (BASE ** 4)
    d0 = v % BASE; v //= BASE
    d1 = v % BASE; v //= BASE
    d2 = v % BASE; v //= BASE
    d3 = v % BASE
    return d0, d1, d2, d3  # low..high

def base251_decode_u32(p1: int, p2: int, p3: int, p4: int) -> int:
    for x in (p1, p2, p3, p4):
        _assert_param(x)
    return (p4 * BASE**3) + (p3 * BASE**2) + (p2 * BASE) + p1

def params_from_monotonic_now_us() -> Tuple[int, int, int, int]:
    ts_us = time.monotonic_ns() // 1000
    return base251_encode_u32(ts_us)

def decode_ts_us_from_params(p1: int, p2: int, p3: int, p4: int) -> int:
    return base251_decode_u32(p1, p2, p3, p4)

# --- retry politika: retry jen při quality chybě (input_err!=0, cmd_err==0) ---
def is_retryable(input_err: int, cmd_err: int) -> bool:
    return (input_err != 0) and (cmd_err == 0)

# --- rate limiter ---
class RateLimiter:
    def __init__(self, min_interval_ms: int = 10):
        self.min_interval_ns = int(min_interval_ms * 1e6)
        self._last_ns = 0
        self._lock = threading.Lock()

    def await_slot(self):
        with self._lock:
            now = time.monotonic_ns()
            wait_ns = self._last_ns + self.min_interval_ns - now
            if wait_ns > 0:
                time.sleep(wait_ns / 1e9)
                now = time.monotonic_ns()
            self._last_ns = now

# --- Ack/Nack manager ---
class AckNackManager:
    """
    - Registruje handlery pro 'IAM' a 'INM' do MessageDispatcher.
    - Matching na (cmd,p1,p2,p3,p4) – cmd je číslo v kódu zprávy: 'IAM50', 'INM50'.
    - Stop-and-Wait: 1 outstanding se stejným obsahem (v praxi stačí, FW nemá frontu).
    """
    def __init__(self, hb, dispatcher, min_interval_ms: int = 10, ack_timeout_ms: int = 20):
        self.hb = hb
        self.dispatcher = dispatcher
        self.rate = RateLimiter(min_interval_ms)
        self.ack_timeout_ms = ack_timeout_ms

        self._lock = threading.Lock()
        self._pending: Dict[CmdKey, Dict] = {}  # key -> {"cond":Condition, "sent_ns":int, "result": AckResult|None}

        # registrace handlerů
        self.dispatcher.register_handler('IAM', self._AckHandler(self))
        self.dispatcher.register_handler('INM', self._NackHandler(self))

    # veřejné API
    def send_and_wait(self, cmd: int, p1: int, p2: int, p3: int, p4: int,
                      timeout_ms: Optional[int] = None,
                      retries: int = 2) -> AckResult:
        if timeout_ms is None:
            timeout_ms = self.ack_timeout_ms

        key: CmdKey = (cmd, p1, p2, p3, p4)
        retries_done = 0
        last_result: Optional[AckResult] = None

        while True:
            # rate-limit
            self.rate.await_slot()

            sent_ns = time.monotonic_ns()
            frame = build_frame(cmd, p1, p2, p3, p4)

            # zaregistruj pending
            with self._lock:
                if key not in self._pending:
                    self._pending[key] = {"cond": threading.Condition(), "sent_ns": sent_ns, "result": None}
                else:
                    self._pending[key]["sent_ns"] = sent_ns
                    self._pending[key]["result"] = None

            if not self.hb.send_frame(frame):
                with self._lock:
                    self._pending.pop(key, None)
                return AckResult(ok=False, is_timeout=False, rtt_ms=0.0,
                                 sent_mono_ns=sent_ns, ack_mono_ns=sent_ns, retries_done=retries_done)

            # čekání na ACK/NACK
            deadline = sent_ns + int(timeout_ms * 1e6)
            result: Optional[AckResult] = None

            with self._lock:
                cond = self._pending[key]["cond"]

            while True:
                now = time.monotonic_ns()
                remain_ns = deadline - now
                if remain_ns <= 0:
                    break
                timeout_s = remain_ns / 1e9
                with cond:
                    cond.wait(timeout_s)
                with self._lock:
                    result = self._pending[key]["result"]
                if result is not None:
                    break

            if result is None:
                # timeout
                retries_done += 1
                last_result = AckResult(
                    ok=False, is_timeout=True, rtt_ms=float((time.monotonic_ns() - sent_ns) / 1e6),
                    sent_mono_ns=sent_ns, ack_mono_ns=time.monotonic_ns(), retries_done=retries_done
                )
                if retries_done > retries:
                    with self._lock:
                        self._pending.pop(key, None)
                    return last_result
                continue  # retry

            # máme ACK/NACK
            result.retries_done = retries_done
            if result.ok:
                with self._lock:
                    self._pending.pop(key, None)
                return result
            else:
                # NACK: retry jen pro quality
                if is_retryable(result.input_err, result.cmd_err) and retries_done < retries:
                    retries_done += 1
                    last_result = result
                    continue
                else:
                    with self._lock:
                        self._pending.pop(key, None)
                    return result

    # vnitřní: dokončení pendingu
    def _complete(self, key: CmdKey, ok: bool, input_err: int, cmd_err: int, ack_ns: int):
        with self._lock:
            pend = self._pending.get(key)
            if not pend:
                return  # pozdní/dup ACK – ignoruj
            sent_ns = pend["sent_ns"]
            rtt_ms = float((ack_ns - sent_ns) / 1e6)
            res = AckResult(ok=ok, is_timeout=False, input_err=input_err, cmd_err=cmd_err,
                            rtt_ms=rtt_ms, sent_mono_ns=sent_ns, ack_mono_ns=ack_ns)
            pend["result"] = res
            cond = pend["cond"]
        with cond:
            cond.notify_all()

    # --- handlery registrované v dispatcheru ---
    class _AckHandler:
        def __init__(self, mgr: 'AckNackManager'):
            self.mgr = mgr
        def handle(self, message_bytes: bytes):
            try:
                code_with_cmd, fields = parse_nmea_line(message_bytes)
                base, cmdnum = split_code_and_cmd(code_with_cmd)
                if base != "IAM" or cmdnum is None:
                    return
                # očekáváme alespoň p1..p4
                if len(fields) < 4:
                    return
                p1, p2, p3, p4 = [int(x) for x in fields[:4]]
                key = (int(cmdnum), p1, p2, p3, p4)
                self.mgr._complete(key, ok=True, input_err=0, cmd_err=0, ack_ns=time.monotonic_ns())
            except Exception:
                pass

    class _NackHandler:
        def __init__(self, mgr: 'AckNackManager'):
            self.mgr = mgr
        def handle(self, message_bytes: bytes):
            try:
                code_with_cmd, fields = parse_nmea_line(message_bytes)
                base, cmdnum = split_code_and_cmd(code_with_cmd)
                if base != "INM" or cmdnum is None:
                    return
                # očekáváme p1..p4 + in_err + cmd_err
                if len(fields) < 6:
                    return
                p1, p2, p3, p4, in_err, cmd_err = [int(x) for x in fields[:6]]
                key = (int(cmdnum), p1, p2, p3, p4)
                self.mgr._complete(key, ok=False, input_err=in_err, cmd_err=cmd_err, ack_ns=time.monotonic_ns())
            except Exception:
                pass

# --- volitelný self-test (periodické CMD=50) ---
if __name__ == '__main__':

    class DummyHandler: 
        def __init__(self, every=10): 
            self.count = 0 
            self.every = every 
        def handle(self, message_bytes: bytes): 
            self.count += 1    

    import signal
    import sys

    try:
        from hb_serial import HoverboardSerial
        from dispatcher import MessageDispatcher
    except Exception as e:
        print("ERROR: Import HoverboardSerial/MessageDispatcher selhal. Ujistěte se, že jsou na PYTHONPATH.")
        sys.exit(1)

    hb = HoverboardSerial()
    hb.start()
    md = MessageDispatcher(hb)
    md.register_handler('ODM', DummyHandler()) 
    md.register_handler('MSM', DummyHandler())
    md.start()

    mgr = AckNackManager(hb, md, min_interval_ms=10, ack_timeout_ms=20)

    stop_evt = threading.Event()
    def _sigint(_sig, _frm): stop_evt.set()
    signal.signal(signal.SIGINT, _sigint)

    CMD_TEST = 0
    period_ns = int(55e6)  # 55 ms
    next_ns = time.monotonic_ns() + period_ns

    sent = ok = nacks = timeouts = 0
    rtt_sum = 0.0
    print("[TEST] Spouštím periodické příkazy CMD=50 každých 55 ms. Ctrl+C pro konec.")
    try:
        while not stop_evt.is_set():
            now = time.monotonic_ns()
            if now < next_ns:
                time.sleep((next_ns - now) / 1e9)
            next_ns += period_ns

            p1, p2, p3, p4 = params_from_monotonic_now_us()
            res = mgr.send_and_wait(CMD_TEST, p1, p2, p3, p4, timeout_ms=20, retries=2)

            sent += 1
            if res.ok:
                ok += 1
                rtt_sum += res.rtt_ms
            elif res.is_timeout:
                timeouts += 1
            else:
                nacks += 1

            if sent % 20 == 0:
                avg = (rtt_sum / ok) if ok else 0.0
                print(f"[TEST] sent={sent} ok={ok} nacks={nacks} timeouts={timeouts} "
                      f"avgRTT={avg:.3f} ms last={res.rtt_ms:.3f} ms "
                      f"(retries={res.retries_done}, INM in_err={res.input_err} cmd_err={res.cmd_err})")
    finally:
        md.stop()
        hb.stop()
