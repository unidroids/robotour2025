import threading
import time

from builders import build_mon_sys_poll
#from builders import build_mon_comms_poll
#from builders import build_esf_status_poll
#from builders import build_esf_raw_poll
#from builders import build_gga_poll

# Konfigurace polleru – přidej/ubírej podle potřeby
POLL_TABLE = [
    {"name": "MON-SYS",   "builder": build_mon_sys_poll},
    #{"name": "MON-COMMS", "builder": build_mon_comms_poll},
    #{"name": "ESF-STATUS","builder": build_esf_status_poll},
    #{"name": "GGA",      "builder": build_gnq_gga_poll},
    #{"name": "ESF-RAW","builder": build_poll_esf_raw},
]


class RotatingPollerThread(threading.Thread):
    def __init__(self, send_func, poll_table=POLL_TABLE, period=5.0):
        """
        send_func: funkce (bytes) → zápis na writer.send_ubx()
        poll_table: list of dicts, např.:
            [
                {"name": "MON-SYS",   "builder": build_mon_sys_poll},
                {"name": "MON-COMMS", "builder": build_mon_comms_poll},
                # další zprávy...
            ]
        period: čas (s) mezi jednotlivými zprávami
        """
        super().__init__(daemon=True)
        self.send_func = send_func
        self.poll_table = poll_table
        self.period = period
        self._stop = threading.Event()
        self._idx = 0

    def stop(self):
        self._stop.set()

    def run(self):
        print("[POLLER] Rotating poller started")
        while not self._stop.is_set():
            row = self.poll_table[self._idx]
            try:
                pkt = row["builder"]()
                self.send_func(pkt)
                print(f"[POLLER] Sent {row['name']}")
            except Exception as e:
                print(f"[POLLER] Error in {row['name']}: {e}")
            self._idx = (self._idx + 1) % len(self.poll_table)
            for _ in range(int(self.period * 10)):
                if self._stop.is_set():
                    break
                time.sleep(0.1)
        print("[POLLER] Rotating poller stopped")