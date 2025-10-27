# drive/ack_nack_test.py
from __future__ import annotations
import signal
import sys
import threading
import time

# Import z vašeho projektu
from ack_nack import AckNackManager, params_from_monotonic_now_us

try:
    from hb_serial import HoverboardSerial
    from dispatcher import MessageDispatcher
except Exception as e:
    print("ERROR: Import HoverboardSerial/MessageDispatcher selhal. Ujistěte se, že jsou na PYTHONPATH.")
    sys.exit(1)

def main():

    class DummyHandler:
        def __init__(self, every=10):
            self.count = 0
            self.every = every

        def handle(self, message_bytes: bytes):
            self.count += 1

    hb = HoverboardSerial()
    hb.start()
    md = MessageDispatcher(hb)
    md.register_handler('ODM', DummyHandler())
    md.register_handler('MSM', DummyHandler())
    md.start()

    mgr = AckNackManager(hb, md, min_interval_ms=10, ack_timeout_ms=20)

    stop_evt = threading.Event()

    def _sigint(_sig, _frm):
        stop_evt.set()

    signal.signal(signal.SIGINT, _sigint)

    CMD_TEST = 50
    period_ns = int(55e6)  # 55 ms
    next_ns = time.monotonic_ns() + period_ns

    sent = ok = nacks = timeouts = 0
    rtt_sum = 0.0
    print("[TEST] Spouštím periodické testovací příkazy CMD=50 každých 55 ms. Ctrl+C pro konec.")

    try:
        while not stop_evt.is_set():
            now = time.monotonic_ns()
            if now < next_ns:
                time.sleep((next_ns - now) / 1e9)
            next_ns += period_ns

            # Timestamp do parametrů (FW vrací echo v IAM/INM)
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

if __name__ == "__main__":
    main()
