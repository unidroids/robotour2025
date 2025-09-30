# service.py
import threading

class Pilot:

    def __init__(self):
        self.running = False
        self._initialized = False
        self._lock = threading.Lock()

    def start(self):
        with self._lock:
            if self.running:
                return "ALREADY_RUNNING"
            if not self._initialized:
                # TODO:init components 

                self._initialized = True
            # TODO: start components

            self.running = True
            print("[SERVICE] STARTED")
            return "OK"

    def stop(self):
        with self._lock:
            if not self.running:
                return "NOT_RUNNING"
            #TODO: stop components

            self._initialized = False
            #TODO: set components to None

            self.running = False
            print("[SERVICE] STOPPED")
            return "OK"

    def _ensure_running(self):
        if not self.running or not self._initialized:
            raise RuntimeError("[PILOT SERVICE] Service is not running. Call START first.")

    # ===== API pro komunikaci s pilotem =====

    #TODO:


