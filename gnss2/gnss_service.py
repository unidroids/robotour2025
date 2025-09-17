# gnss_service.py
# -*- coding: utf-8 -*-
import argparse
import logging
import queue
import struct
import sys
import threading
import time
from collections import Counter, defaultdict

import serial

from ubx_proto import SerialBytes, parse_stream, build_poll
from gnss_config import make_initial_config, apply_config_with_ack
from itow_extract import extract_itow  # ← soubor itow_extract.py ulož vedle tohoto

# ----- UBX Class/ID (správné hodnoty) -----
CLASS_NAV = 0x01
CLASS_ESF = 0x10
CLASS_MON = 0x0A

ID_NAV_ATT      = 0x05  # UBX-NAV-ATT
ID_NAV_VELNED   = 0x12  # UBX-NAV-VELNED
ID_NAV_HPPOSLLH = 0x14  # UBX-NAV-HPPOSLLH (iTOW na offsetu 4!)
ID_NAV_EOE      = 0x61  # UBX-NAV-EOE

ID_ESF_INS      = 0x15  # UBX-ESF-INS
ID_ESF_STATUS   = 0x10  # UBX-ESF-STATUS (poll/periodic)

ID_MON_TXBUF    = 0x08  # UBX-MON-TXBUF (poll/periodic)
ID_MON_SYS      = 0x39  # UBX-MON-SYS   (poll/periodic)

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


def decode_mon_sys(payload: bytes) -> dict:
    """
    UBX-MON-SYS (0x0A 0x39), běžná délka 24 B (msgVer=1).
    Pole: cpu/mem/io usage (%), runtime, počty notice/warn/error, teplota.
    """
    if len(payload) < 19:
        return {"len": len(payload), "raw_hex": payload.hex()}
    # <BBBBBBBB = msgVer, bootType, cpuLoad, cpuLoadMax, memUsage, memUsageMax, ioUsage, ioUsageMax
    msgVer, bootType, cpuLoad, cpuLoadMax, memUsage, memUsageMax, ioUsage, ioUsageMax = struct.unpack_from(
        "<BBBBBBBB", payload, 0
    )
    runTime = struct.unpack_from("<I", payload, 8)[0]
    notice, warn, err = struct.unpack_from("<HHH", payload, 12)
    temp = struct.unpack_from("<b", payload, 18)[0]
    return {
        "msgVer": msgVer,
        "bootType": bootType,
        "cpuLoad%": cpuLoad,
        "cpuLoadMax%": cpuLoadMax,
        "memUsage%": memUsage,
        "memUsageMax%": memUsageMax,
        "ioUsage%": ioUsage,
        "ioUsageMax%": ioUsageMax,
        "runTime_s": runTime,
        "notice": notice,
        "warn": warn,
        "err": err,
        "temp_C": temp,
    }


def decode_mon_txbuf(payload: bytes) -> dict:
    """
    UBX-MON-TXBUF (0x0A 0x08), délka 28 B.
    pending[6] (U2), usage[6] % (U1), peak[6] % (U1), totalUsage (U1), totalPeak (U1), errors (U1).
    """
    if len(payload) < 28:
        return {"len": len(payload), "raw_hex": payload.hex()}
    pending = list(struct.unpack_from("<6H", payload, 0))
    usage = list(struct.unpack_from("<6B", payload, 12))
    peak = list(struct.unpack_from("<6B", payload, 18))
    tUsage, tPeak, errors = struct.unpack_from("<BBB", payload, 24)
    return {
        "pending": pending,
        "usage%": usage,
        "peak%": peak,
        "totalUsage%": tUsage,
        "totalPeak%": tPeak,
        "errors": errors,
    }


class GNSSService:
    def __init__(self, port: str, baud: int, log_path: str, prio_hz: int):
        self.port = port
        self.baud = baud
        self.prio_hz = prio_hz

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

        # epoch collector: iTOW -> set(msg_class,id)
        self.epoch_map: dict[int, set[tuple[int, int]]] = {}
        self.last_eoe_itow: int | None = None

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
        for cls, mid, payload, ts in parse_stream(iter(byte_iter)):
            try:
                self.recv_q.put((cls, mid, payload, ts), timeout=0.1)
            except queue.Full:
                self.log.warning("recv_q FULL – dropping message")

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
                # MON-TXBUF
                self.send_q.put(build_poll(CLASS_MON, ID_MON_TXBUF))
            elif sel == 1:
                # ESF-STATUS
                self.send_q.put(build_poll(CLASS_ESF, ID_ESF_STATUS))
            else:
                # MON-SYS
                self.send_q.put(build_poll(CLASS_MON, ID_MON_SYS))
            i += 1
            # zarovnání na sekundy
            t = time.time()
            time.sleep(max(0.0, 1.0 - (t - int(t))))

    # ----- high-level -----
    def apply_config(self):
        # sestavení a odeslání configu + čekání na ACK
        frame = make_initial_config(prio_hz=self.prio_hz, enable_nav_eoe=True)
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
                try:
                    cls, mid, payload, ts = self.recv_q.get(timeout=0.2)
                except queue.Empty:
                    pass
                else:
                    self.handle_message(cls, mid, payload)

                # per-second stats
                now = time.time()
                if now - self.last_stats >= 1.0:
                    self.last_stats = now
                    if self.counts:
                        s = ", ".join(
                            f"{k[0]:02X}/{k[1]:02X}:{v}" for k, v in sorted(self.counts.items())
                        )
                    else:
                        s = "-"
                    self.log.info("Counts: %s", s)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_ev.set()
            try:
                # na některých platformách existuje cancel_read(); když není, prostě zavři
                if hasattr(self.ser, "cancel_read"):
                    self.ser.cancel_read()
            except Exception:
                pass            
            try:
                self.ser.close()
            except Exception:
                pass
            
            # nech vlákna doběhnout krátkým joinem, pokud nejsou daemon
            #for t in self.threads: t.join(timeout=0.5)

    # ----- message handlers -----
    def handle_message(self, cls: int, mid: int, payload: bytes):
        self.counts[(cls, mid)] += 1

        # decode MON messages for visibility
        if (cls, mid) == (CLASS_MON, ID_MON_TXBUF):
            info = decode_mon_txbuf(payload)
            self.log.info("MON-TXBUF: %s", info)
            return
        elif (cls, mid) == (CLASS_MON, ID_MON_SYS):
            info = decode_mon_sys(payload)
            self.log.info("MON-SYS: %s", info)
            return

        # collect epoch parts (PRIO set + EOE)
        if (cls, mid) in (NEEDED | {(CLASS_NAV, ID_NAV_EOE)}):
            itow = extract_itow((cls, mid), payload)
            if itow is None:
                # unexpected – log once in a while
                if self.counts[(cls, mid)] % 10 == 1:
                    self.log.warning("No iTOW for %s", MSG_NAMES.get((cls, mid), f"{cls:02X}/{mid:02X}"))
                return

            if (cls, mid) == (CLASS_NAV, ID_NAV_EOE):
                # flush epoch
                present = self.epoch_map.pop(itow, set())
                missing = NEEDED - present
                if missing:
                    # přelož na názvy
                    missing_n = {MSG_NAMES.get(m, f"{m[0]:02X}/{m[1]:02X}") for m in missing}
                    self.log.warning("Epoch %d: chybí %s", itow, missing_n)
                else:
                    self.log.debug("Epoch %d: kompletní PRIO set", itow)
                self.last_eoe_itow = itow
            else:
                s = self.epoch_map.setdefault(itow, set())
                s.add((cls, mid))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--prio", type=int, default=10, help="PRIO výstupní frekvence (0..30 Hz)")
    ap.add_argument("--log", default="gnss.log")
    args = ap.parse_args()

    svc = GNSSService(args.port, args.baud, args.log, args.prio)
    try:
        svc.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
