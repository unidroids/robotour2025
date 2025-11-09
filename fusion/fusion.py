# fusion.py
from __future__ import annotations
import threading
import time
import math 
import traceback
from dataclasses import dataclass, asdict
from typing import Optional, Tuple


@dataclass
class FusionState:
    mode: str = "IDLE"                 # IDLE | WAITING | READY
    last_note: str = ""
    ts_mono: float = 0.0               # monotonic timestamp poslední aktualizace

class Fusion:

    VERSION = "1.0.1"

    def __init__(self):
        self.running = False
        self._initialized = False
        self._lock = threading.Lock()
        #self._stop_event = threading.Event()
        #self._thread: Optional[threading.Thread] = None

        self._state_lock = threading.Lock()
        self._state = FusionState()

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

    def start(self):
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

    def stop(self):
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

  

    # ---------------------- events -----------------



if __name__ == "__main__":
    print(f"TEST") 
    fusion = Fusion()
    fusion.start()
    print(fusion.get_state())
    fusion.stop()
    print(fusion.get_state())
    # try:
    #     while True:
    #         time.sleep(1.0)
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     fusion.stop()