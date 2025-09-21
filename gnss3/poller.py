import threading
import time

from builders import build_mon_sys_poll, build_mon_comms_poll

default_poll_table = [
    {"name": "MON-SYS",   "builder": build_mon_sys_poll,   "interval": 5.0},
    #{"name": "MON-COMMS", "builder": build_mon_comms_poll, "interval": 10.0},
    # atd. přidáš další, pokud potřebuješ
]

class RotatingPollerThread(threading.Thread):
    def __init__(self, send_func, poll_table=default_poll_table, period=10.0):
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