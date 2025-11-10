from __future__ import annotations
import socket
import time
import threading
from typing import Optional

from data.nav_pvat_data import NavPvatData  # type: ignore
from data.nav_fusion_data import NavFusionData  # type: ignore



# --- fusion of different gnss senteces ------------------------------------------------------

class NavFusion:
    """
    Fúzní engine pro seskládání gnss vět.

    Vstupy (10 Hz):
      - GNSS PVAT: obsahuje (motHeading nebo vehHeading/heading), rychlost, kvality
      - Future solution:
        - UBX-NAV-HPPOSECEF
        - UBX-NAV-VELECEF
        - UBX-NAV-EOE

    """

    def __init__(self, host = "127.0.0.1", port=9009):
        # target service Pilot
        self.host = host
        self.port = port
        self.sock = None

        # Stav
        self._latest: Optional[NavFusionData] = None
        self._latest_lock = threading.Lock()
        self._cond = threading.Condition()




    # === Vstup z NAV-PVAT handleru ==========================================
    def on_nav_pvat(self, pvat: NavPvatData) -> None:
        now = time.monotonic()

        # Zde nastav další pole podle dostupných dat
        fusion_data = NavFusionData(
            ts_mono=now,
            lat=pvat.lat,
            lon=pvat.lon,
            hAcc=pvat.hAcc,
            heading=pvat.motHeading,
            headingAcc=pvat.accHeading,
            speed= pvat.gSpeed,
            sAcc=pvat.sAcc,
            gyroZ=0.0, # dummy values,
            gyroZAcc=0.0, # dummy values,
            gnssFixOK=int(pvat.carrSoln in (2, 3)),  # fix s GNSS
            drUsed=int(pvat.fixType in (4, 5)),  # dead reckoning
        )
        self._publish(fusion_data)
        self._push_nav_solution()

    # === Odběratelské API ====================================================

    def get_latest(self) -> Optional[NavFusionData]:
        with self._latest_lock:
            return self._latest

    def wait_for_update(self, timeout: Optional[float] = None) -> bool:
        with self._cond:
            return self._cond.wait(timeout=timeout)

    # === Interní =============================================================

    def _publish(self, res: NavFusionData) -> None:
        with self._latest_lock:
            self._latest = res
        with self._cond:
            self._cond.notify_all()

    # === Push =============================================================

    def _connect(self):
        self.sock = socket.create_connection((self.host, self.port))
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def _reconnect(self):
        retries = 0
        while not self.sock:
            self._connect()
            time.sleep(0.5)
            retries += 1
            if retries >= 5:
                print("[DRIVE CLIENT] Failed to reconnect after 5 attempts.")
                raise ConnectionError("Unable to reconnect to drive server.")
                
    def _push_nav_solution(self):
        self._reconnect()
        nav = self.get_latest()
        self.sock.sendall(b'GNSS\n' + nav.to_bytes())
        

# --- self-test ---
if __name__ == "__main__":
    sol = NavFusionData(
        ts_mono=12345.678,
        lat=49.0001234,
        lon=17.0005678,
        hAcc=0.25,
        heading=92.4,
        headingAcc=1.2,
        speed=0.54,
        sAcc=0.05,
        gyroZ=-12.3,
        gyroZAcc=0.8,
        gnssFixOK=True,
        drUsed=False,
    )
    nav = NavFusion()
    nav._publish(sol)
    nav._push_nav_solution()
    #time.sleep(1)
