# pilot.py
import threading
import time 
import traceback

from drive_client import DriveClient
from gnss_client import GnssClient
from data.nav_fusion_data import NavFusionData

class Pilot:

    def __init__(self):
        self.running = False
        self._initialized = False
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None
        self.gnss_client = None
        self.drive_client = None
        self.target = None  # (start, goal, radius)

    def start(self):
        with self._lock:
            if self.running:
                return "ALREADY_RUNNING"
            if not self._initialized:
                self.drive_client = DriveClient()
                self.gnss_client = GnssClient()
                self._initialized = True

            self.drive_client.start()
            self.gnss_client.start()

            self.running = True
            print("[SERVICE] STARTED")
            return "OK"

    def stop(self):
        with self._lock:
            if not self.running:
                return "NOT_RUNNING"
            self._stop_event.set()
            if self._thread:
                self._thread.join(timeout=2.0)
            self.gnss_client.stop()
            self.drive_client.stop()
            self.drive_client = None
            self.gnss_client = None
            self._initialized = False
            self.running = False
            print("[SERVICE] STOPPED")
            return "OK"

    def _ensure_running(self):
        if not self.running or not self._initialized:
            raise RuntimeError("[PILOT SERVICE] Service is not running. Call START first.")

    def _navigate_thread(self, gnss, drive, start, goal, radius):
        left_pwm = 0
        right_pwm = 0
        print(f"[PILOT] Navigation started: from {start} to {goal} within radius {radius}m")
        while not self._stop_event.is_set():
            try:
                nav_data = gnss.read_nav_fusion_data()
                if nav_data:
                    print(f"[PILOT] Nav data: {nav_data}")
                    drive.send_pwm(left_pwm, right_pwm) 
                else:
                    drive.send_pwm(0, 0)  # Stop if no data
                    time.sleep(1) # Wait before retrying
            except Exception as e:
                drive.send_pwm(0, 0)  # Stop on error
                print(f"[PILOT ERROR] {e}")
                traceback.print_exc()
                time.sleep(1) # Wait before retrying
            

            
    # ===== API pro komunikaci s pilotem =====

    def navigate(self, start, goal, radius):
        gnss = self.gnss_client
        drive = self.drive_client
        with self._lock:
            self._ensure_running()
            if self._thread and self._thread.is_alive():
                self._stop_event.set()
                self._thread.join()
            self.target = (start, goal, radius)
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._navigate_thread, args=(gnss, drive, start, goal, radius,), daemon=True)
            self._thread.start()

        
        


