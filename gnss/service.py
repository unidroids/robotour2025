# service.py
from service_start import init_gnss_service
import threading

class GnssService:

    def __init__(self):
        self.running = False
        self._initialized = False
        self._lock = threading.Lock()

    def start(self):
        with self._lock:
            if self.running:
                return "ALREADY_RUNNING"
            if not self._initialized:
                components = init_gnss_service()
                self.__dict__.update(components)
                self._initialized = True
            self.gnss.open()
            self.nmea_dispatcher.start()
            self.ubx_dispatcher.start()
            self.poller.start()
            self.running = True
            print("[SERVICE] STARTED")
            return "OK"

    def stop(self):
        with self._lock:
            if not self.running:
                return "NOT_RUNNING"
            if self.poller:
                self.poller.stop()
            if self.gnss:
                self.gnss.close()
            if self.ubx_dispatcher:
                self.ubx_dispatcher.stop()
            if self.nmea_dispatcher:
                self.nmea_dispatcher.stop()
            self.nmea_dispatcher = None
            self.ubx_dispatcher = None
            self.gnss = None
            self.poller = None
            self.running = False
            print("[SERVICE] STOPPED")
            return "OK"

    def _ensure_running(self):
        if not self.running or not self._initialized:
            raise RuntimeError("[GNSS SERVICE] Service is not running. Call START first.")

    def get_gga(self):
        self._ensure_running()
        return self.nmea_gga_handler.get_last_gga()

    def get_data_json(self):
        self._ensure_running()
        import json
        res = self.nav_fusion.get_latest()
        if not res:
            return "{}"
        obj = {
            "iTOW": res.iTOW_ms,
            "heading_deg": res.heading_deg,
            "speed_mps": res.speed_mps,
            "quality": res.quality,
            "t_mono": res.t_mono,
        }
        return json.dumps(obj)

    def get_nav_fusion(self):
        self._ensure_running()
        return self.nav_fusion

    def get_latest_data(self):
        self._ensure_running()
        return self.get_data_json()
