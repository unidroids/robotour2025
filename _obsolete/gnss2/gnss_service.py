# gnss_service.py
# -*- coding: utf-8 -*-
import argparse
import logging
import queue
import statistics
import struct
import sys
import threading
import time
from collections import Counter, defaultdict, deque

import serial

from ubx_proto import SerialBytes, parse_stream, build_poll
from gnss_config import make_initial_config, apply_config_with_ack
from itow_extract import extract_itow
from ubx_decode import decode_mon_sys, decode_mon_txbuf  # ← přesunuto sem

# ----- UBX Class/ID -----
CLASS_NAV = 0x01
CLASS_ESF = 0x10
CLASS_MON = 0x0A

ID_NAV_ATT      = 0x05  # UBX-NAV-ATT
ID_NAV_VELNED   = 0x12  # UBX-NAV-VELNED
ID_NAV_HPPOSLLH = 0x14  # UBX-NAV-HPPOSLLH (iTOW @ +4)
ID_NAV_EOE      = 0x61  # UBX-NAV-EOE

ID_ESF_INS      = 0x15  # UBX-ESF-INS
ID_ESF_STATUS   = 0x10  # UBX-ESF-STATUS (poll/periodic)

ID_MON_TXBUF    = 0x08  # UBX-MON-TXBUF
ID_MON_SYS      = 0x39  # UBX-MON-SYS

# „povinná“ PRIO sada v rámci jednoho iTOW
NEEDED = {
    (CLASS_NAV, ID_NAV_ATT),
    (CLASS_NAV, ID_NAV_VELNED),
    (CLASS_NAV, ID_NAV_HPPOSLLH),
    (CLASS_ESF, ID_ESF_INS),
}

MSG_NAMES = {
    (CLASS_NAV, ID_NAV_ATT): "NAV-ATT",
    (CLASS_NAV, ID_NAV_VELNED): "NAV-VELNED",
    (CLASS_NAV, ID_NAV_HPPOSLLH): "NAV-HPPOSLLH",
    (CLASS_NAV, ID_NAV_EOE): "NAV-EOE",
    (CLASS_ESF, ID_ESF_INS): "ESF-INS",
    (CLASS_ESF, ID_ESF_STATUS): "ESF-STATUS",
    (CLASS_MON, ID_MON_TXBUF): "MON-TXBUF",
    (CLASS_MON, ID_MON_SYS): "MON-SYS",
}


def quantiles_safe(samples, qs):
    """Jednoduché kvantily bez numpy; vrací dict q->value, prázdné pokud vzorků málo."""
    if not samples:
        return {q: None for q in qs}
    data = sorted(samples)
    out = {}
    n = len(data)
    for q in qs:
        if n == 1:
            out[q] = data[0]
            continue
        # p-th quantile index (inclusive lower)
        pos = q * (n - 1)
        lo = int(pos)
        hi = min(lo + 1, n - 1)
        frac = pos - lo
        out[q] = data[lo] * (1 - frac) + data[hi] * frac
    return out


class GNSSService:
    def __init__(self, port: str, baud: int, log_path: str, prio_hz: int,
                 bucket_timeout_ms: int | None = None, span_hist_size: int = 300):
        self.port = port
        self.baud = baud
        self.prio_hz = prio_hz
        self.bucket_timeout_ms = bucket_timeout_ms  # pokud None, timeout se nepoužije

        # serial non-blocking
        self.ser = serial.Serial(port, baudrate=baud, timeout=0, write_timeout=0)

        # queues
        self.recv_q: "queue.Queue[tuple[int,int,bytes,float]]" = queue.Queue(maxsize=10000)
        self.send_q: "queue.Queue[bytes]" = queue.Queue(maxsize=10000)

        # control
        self.stop_ev = threading.Event()

        # stats
        self.counts = Counter()
        self.last_stats = time.time()
        self.span_hist = deque(maxlen=span_hist_size)  # ms z kompletních epoch (first→last)

        # epoch collector: iTOW -> dict
        #   { 'present': set[(cls,id)],
        #     'first_ts': float,  # monotonic čas 1. zprávy v bucketu
        #     'last_ts':  float,  # monotonic čas poslední zprávy v bucketu
        #     'complete_logged': bool }
        self.epoch_map: dict[int, dict] = {}
        self.first_eoe_seen = False

        # logging
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s %(levelname)s %(message)s",
            handlers=[logging.FileHandler(log_path), logging.StreamHandler(sys.stdout)],
        )
        self.log = logging.getLogger("gnss")

    # ----- threads -----
    def start_threads(self):
        threading.Thread(target=self.reader_loop, name="ubx-reader", daemon=True).start()
        threading.Thread(target=self.writer_loop, name="ubx-writer", daemon=True).start()
        threading.Thread(target=self.poller_loop, name="poller", daemon=True).start()

    def reader_loop(self):
        byte_iter = SerialBytes(self.ser)
        try:
            for cls, mid, payload, ts in parse_stream(iter(byte_iter)):
                if self.stop_ev.is_set():
                    break
                try:
                    self.recv_q.put((cls, mid, payload, ts), timeout=0.1)
                except queue.Full:
                    self.log.warning("recv_q FULL – dropping message")
        except (serial.SerialException, OSError) as e:
            if not self.stop_ev.is_set():
                self.log.warning("reader stopped: %s", e)

    def writer_loop(self):
        while not self.stop_ev.is_set():
            try:
                data = self.send_q.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                self.ser.write(data)
                self.ser.flush()
            except serial.SerialTimeoutException:
                self.log.warning("serial write timeout")

    def poller_loop(self):
        i = 0
        while not self.stop_ev.is_set():
            sel = i % 3
            if sel == 0:
                self.send_q.put(build_poll(CLASS_MON, ID_MON_TXBUF))
            elif sel == 1:
                self.send_q.put(build_poll(CLASS_ESF, ID_ESF_STATUS))
            else:
                self.send_q.put(build_poll(CLASS_MON, ID_MON_SYS))
            i += 1
            # zarovnání na sekundy
            t = time.time()
            time.sleep(max(0.0, 1.0 - (t - int(t))))

    # ----- high-level -----
    def apply_config(self):
        # make_initial_config může vracet 1, 2 nebo 3 hodnoty
        res = make_initial_config(prio_hz=self.prio_hz, enable_nav_eoe=True)

        frame = None
        cfg_items = None
        # možnost: res je tuple s 1..3 prvky, nebo už přímo bytes
        if isinstance(res, tuple):
            n = len(res)
            if n >= 1:
                frame = res[0]
            if n >= 2:
                cfg_items = res[1]
            # pokud je n >= 3, třetí položku ignorujeme (meta/debug)
        else:
            frame = res  # res je přímo frame (bytes)

        if cfg_items is not None:
            self.log.info("Config items: %s", cfg_items)

        ok = apply_config_with_ack(self.ser, self.recv_q, frame, timeout=1.5)
        if not ok:
            self.log.error("CFG-VALSET nebyl ACKnut! Zkontroluj klíče/port.")
        else:
            self.log.info("CFG-VALSET ACK 👍")

    def run(self):
        self.start_threads()
        time.sleep(0.1)
        self.apply_config()

        self.log.info("Running on %s @ %d, PRIO=%d Hz", self.port, self.baud, self.prio_hz)

        try:
            while not self.stop_ev.is_set():
                # příjem
                try:
                    cls, mid, payload, ts = self.recv_q.get(timeout=0.2)
                except queue.Empty:
                    pass
                else:
                    self.handle_message(cls, mid, payload, ts)

                # housekeeping: per-second stats + případné timeouts
                now = time.time()
                if now - self.last_stats >= 1.0:
                    self.last_stats = now
                    # optional timeout flush
                    if self.bucket_timeout_ms is not None:
                        self.flush_timeouts(now_monotonic=time.monotonic())

                    # stats line
                    if self.counts:
                        s = ", ".join(
                            f"{k[0]:02X}/{k[1]:02X}:{v}" for k, v in sorted(self.counts.items())
                        )
                    else:
                        s = "-"
                    # span stats
                    qs = quantiles_safe(list(self.span_hist), [0.5, 0.95, 0.99])
                    span_txt = (
                        f"spans_ms(n={len(self.span_hist)}): "
                        f"p50={qs[0.5]:.1f} p95={qs[0.95]:.1f} p99={qs[0.99]:.1f}"
                        if len(self.span_hist) >= 5 else
                        f"spans_ms(n={len(self.span_hist)}): collecting…"
                    )
                    self.log.info("Counts: %s | %s", s, span_txt)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_ev.set()
            try:
                if hasattr(self.ser, "cancel_read"):
                    self.ser.cancel_read()
            except Exception:
                pass
            try:
                self.ser.close()
            except Exception:
                pass

    # ----- message handlers / aggregator -----
    def handle_message(self, cls: int, mid: int, payload: bytes, ts_mono: float):
        self.counts[(cls, mid)] += 1

        # Decode MON messages for visibility
        if (cls, mid) == (CLASS_MON, ID_MON_TXBUF):
            info = decode_mon_txbuf(payload)
            self.log.info("MON-TXBUF: %s", info)
            return
        elif (cls, mid) == (CLASS_MON, ID_MON_SYS):
            info = decode_mon_sys(payload)
            self.log.info("MON-SYS: %s", info)
            return

        # PRIO + EOE kolektor
        is_tracked = (cls, mid) in NEEDED or (cls, mid) == (CLASS_NAV, ID_NAV_EOE)
        if not is_tracked:
            print("not recognized msg", cls, mid)
            return

        itow = extract_itow((cls, mid), payload)
        if itow is None:
            # jen zřídka — logni občas
            if self.counts[(cls, mid)] % 50 == 1:
                self.log.warning("No iTOW for %s", MSG_NAMES.get((cls, mid), f"{cls:02X}/{mid:02X}"))
            return

        if (cls, mid) == (CLASS_NAV, ID_NAV_EOE):
            if not self.first_eoe_seen:
                self.first_eoe_seen = True
                # první EOE nevalidujeme, jen watermark
                return
            # uzavři vše s iTOW <= EOE (watermark)
            self.flush_up_to_eoe(itow)
            return

        # PRIO zpráva: aktualizuj bucket
        b = self.epoch_map.get(itow)
        if b is None:
            b = self.epoch_map[itow] = {
                "present": set(),
                "first_ts": ts_mono,
                "last_ts": ts_mono,
                "complete_logged": False,
            }
        b["present"].add((cls, mid))
        if ts_mono > b["last_ts"]:
            b["last_ts"] = ts_mono

        # když se sada uzavře, spočítej span a ulož do hist
        if not b["complete_logged"] and b["present"] >= NEEDED:
            span_ms = (b["last_ts"] - b["first_ts"]) * 1000.0
            self.span_hist.append(span_ms)
            b["complete_logged"] = True
            # detailní log pro daný iTOW
            # self.log.info(
            #     "Epoch %d COMPLETE: span_ms=%.1f (from %s) ",
            #     itow, span_ms, ", ".join(sorted(MSG_NAMES[m] for m in b["present"]))
            # )

    def flush_up_to_eoe(self, eoe_itow: int):
        """Uzavři všechny buckety s iTOW <= eoe_itow. Pokud nejsou kompletní, zaloguj co chybí."""
        to_delete = []
        for itow, b in self.epoch_map.items():
            if itow <= eoe_itow:
                missing = NEEDED - b["present"]
                if missing:
                    span_ms = (b["last_ts"] - b["first_ts"]) * 1000.0
                    self.log.warning(
                        "Epoch %d INCOMPLETE: missing=%s, span_ms=%.1f, have=%s",
                        itow,
                        {MSG_NAMES.get(m, f"{m[0]:02X}/{m[1]:02X}") for m in missing},
                        span_ms,
                        {MSG_NAMES.get(m, f"{m[0]:02X}/{m[1]:02X}") for m in b["present"]},
                    )
                to_delete.append(itow)
        for itow in to_delete:
            self.epoch_map.pop(itow, None)

    def flush_timeouts(self, now_monotonic: float):
        """Volitelně: force-flush bucketů starších než timeout (od first_ts)."""
        if self.bucket_timeout_ms is None:
            return
        timeout_s = self.bucket_timeout_ms / 1000.0
        to_delete = []
        for itow, b in self.epoch_map.items():
            if now_monotonic - b["first_ts"] >= timeout_s:
                missing = NEEDED - b["present"]
                span_ms = (b["last_ts"] - b["first_ts"]) * 1000.0
                if missing:
                    self.log.warning(
                        "Epoch %d TIMEOUT: missing=%s, span_ms=%.1f, have=%s",
                        itow,
                        {MSG_NAMES.get(m, f"{m[0]:02X}/{m[1]:02X}") for m in missing},
                        span_ms,
                        {MSG_NAMES.get(m, f"{m[0]:02X}/{m[1]:02X}") for m in b["present"]},
                    )
                else:
                    # teoreticky už zalogováno při COMPLETE, ale kdyby ne:
                    if not b["complete_logged"]:
                        self.span_hist.append(span_ms)
                        self.log.info("Epoch %d COMPLETE(timeout): span_ms=%.1f", itow, span_ms)
                to_delete.append(itow)
        for itow in to_delete:
            self.epoch_map.pop(itow, None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/gnss1")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--prio", type=int, default=30, help="PRIO výstupní frekvence (0..30 Hz)")
    ap.add_argument("--log", default="gnss.log")
    ap.add_argument("--bucket-w-ms", type=int, default=None,
                    help="Volitelný timeout bucketu (ms) od první zprávy; None = nepoužívat")
    args = ap.parse_args()

    svc = GNSSService(args.port, args.baud, args.log, args.prio, bucket_timeout_ms=args.bucket_w_ms)
    svc.run()


if __name__ == "__main__":
    main()
