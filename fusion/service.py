# fusion.py

from __future__ import annotations
import threading
import time
from dataclasses import dataclass, asdict
from typing import Optional, Tuple

from data.nav_fusion_data import NavFusionData
from core import FusionCore

__all__ = [
    "FusionService"
]


@dataclass
class FusionState:
    mode: str = "IDLE"                 # IDLE | WAITING | READY
    last_note: str = ""
    ts_mono: float = 0.0               # monotonic timestamp poslední aktualizace

class FusionService:

    VERSION = "1.0.1"

    def __init__(self):
        self.running = False
        self._initialized = False
        self._lock = threading.Lock()
        #self._stop_event = threading.Event()
        #self._thread: Optional[threading.Thread] = None

        self._state_lock = threading.Lock()
        self._state = FusionState()

        self._data_lock = threading.Lock()

        self.LIDAR_MESSAGE_LENGHT = 1
        self.GNSS_MESSAGE_LENGHT = NavFusionData.byte_size()
        self.CAMERA_MESSAGE_LENGHT = 1
        self.DRIVE_MESSAGE_LENGHT = 0

        # latest data
        self.core = FusionCore()
        self.last_gnss_data = None
        self.last_heading_data = None
        self.last_drive_data = None

        # auto start
        self._start()

    # ---------------------- stavové API ----------------------

    def _set_state(self, **updates) -> None:
        with self._state_lock:
            for k, v in updates.items():
                setattr(self._state, k, v)
            self._state.ts_mono = time.monotonic()

    def get_state(self) -> dict:
        with self._state_lock:
            return asdict(self._state)

    # ---------------------- lifecycle ------------------------

    def _start(self):
        with self._lock:
            if self.running:
                return "OK ALREADY_RUNNING"
            if not self._initialized:
                #TODO init helpers
                self._initialized = True
            self.running = True
            self._set_state(mode="WAITING", last_note="SERVICE STARTED")
            print("[SERVICE] STARTED")
            return "OK"

    def _stop(self):
        with self._lock:
            if not self.running:
                return "OK WAS NOT RUNNING"
            try:
                #TODO
                pass
            finally:
                #TODO
                self._initialized = False
                self.running = False
                self._set_state(mode="IDLE", last_note="SERVICE STOPPED")
                print("[SERVICE] STOPPED")
            return "OK"

    def _ensure_running(self):
        if not self.running or not self._initialized:
            raise RuntimeError("[FUSION SERVICE] Service is not running. Call START first.")

  
    def restart(self):
        self._stop()
        self._start()


    # ---------------------- events -----------------
    def on_gnss_data(self, msg):
        #print("on_gnss_data", msg)
        nav = NavFusionData.from_bytes(msg)
        self.last_gnss_data = nav
        self.core.update_position(
            iTow=nav.ts_mono,
            lat=nav.lat,
            lon=nav.lon,
            hAcc=nav.hAcc,
            height=0.0, #TODO get height from gnss
            vAcc=1.0, #TODO get vAcc from gnss
        )

    def on_drive_data(self, msg):
        #print("on_drive_data", msg)
        self.last_drive_data = msg
        (tmark, omega, angle, left_speed, right_speed) = parse_drive_msg(msg)
        self.core.update_whell_speed(
            tmark=tmark,
            left_wheel_speed=left_speed,
            right_wheel_speed=right_speed,
        )
        self.core.update_local_heading(
            tmark=tmark,
            heading=angle,
            omega=omega,
        )

    def on_heading_data(self, msg):
        #print("on_heading_data", msg)
        self.last_heading_data = msg
        (sol, pos, length, heading, pitch, reserved, hdgstddev, ptchstddev) = parse_heading_msg(msg)
        self.core.update_global_roll(
            iTow=0.0, #TODO
            lenght=length,
            roll=pitch,
            gstddev=ptchstddev,
        )
        self.core.update_global_heading(
            iTow=0.0, #TODO
            lenght=length,
            heading=heading,
            gstddev=hdgstddev,
        )


    def on_camera_data(self, msg):
        print("on_camera_data", msg)
        pass

    def on_lidar_data(self, msg):
        print("on_lidar_data", msg)
        pass
    
    # -------------- datové API -------------
    def get_solution(self):
        if not self.core.ready: return None



    # -------------- push to pilot ---------

    def publish(self):
        print(f"[EngineCore] Publish at {time.monotonic}")



def parse_drive_msg(msg):
    # ASCII zpráva: "tmark,omega,angle,left_speed,right_speed"
    # Vstup může být bytes nebo str. Naivní konverze bez kontrol.

    if isinstance(msg, bytes):
        msg = msg.decode('ascii', errors='replace')

    parts = msg.strip().split(',')

    if len(parts) != 5: 
        raise ValueError(f"parse_drive_msg: expected 8 fields, got {len(parts)}")

    tmark       = int(parts[0], 10)
    omega       = int(parts[1], 10)       # může být ±
    angle       = int(parts[2], 10)       # může být ±
    left_speed  = int(parts[3], 10)       # může být ±
    right_speed = int(parts[4], 10)       # může být ±

    return (tmark, omega, angle, left_speed, right_speed)


def parse_heading_msg(msg):
    """
    ASCII/bytes zpráva: "sol,pos,length,heading,pitch,reserved,hdgstddev,ptchstddev"
    – Oddělovač: čárka ','
    – Desetinná tečka '.'
    – Typy: sol=text, pos=text, ostatní=float
    – Pouze kontrola počtu položek, bez rozsahových validací
    """
    if isinstance(msg, bytes):
        msg = msg.decode('ascii', errors='replace')

    parts = [p.strip() for p in msg.strip().split(',')]
    if len(parts) != 8:
        raise ValueError(f"parse_heading_msg: expected 8 fields, got {len(parts)}")

    sol         = parts[0]   # Solution status (enum)
    pos         = parts[1]    # Position type (enum)
    length      = float(parts[2])      # baseline length
    heading     = float(parts[3])      # 0..360
    pitch       = float(parts[4])      # -90..+90
    reserved    = float(parts[5])
    hdgstddev   = float(parts[6])
    ptchstddev  = float(parts[7])

    return (sol, pos, length, heading, pitch, reserved, hdgstddev, ptchstddev)


if __name__ == "__main__":
    print(f"TEST") 
    fusion = FusionService()
    #fusion.start()
    print(fusion.get_state())
    fusion.restart()
    print(fusion.get_state())
    # try:
    #     while True:
    #         time.sleep(1.0)
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     fusion.stop()