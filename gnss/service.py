# service.py
from service_start import init_gnss_service
from builders import build_odm
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
            self._initialized = False
            self.nmea_dispatcher = None
            self.ubx_dispatcher = None
            self.gnss = None
            self.poller = None
            self.handlers = None
            self.running = False
            print("[SERVICE] STOPPED")
            return "OK"

    def _ensure_running(self):
        if not self.running or not self._initialized:
            raise RuntimeError("[GNSS SERVICE] Service is not running. Call START first.")


    # API pro získání poslední GGA věty
    def get_gga(self):
        self._ensure_running()
        return self.handlers['nmea_gga'].get_last_gga()

    # API pro získání dat z NavFusion
    def wait_for_update(self, timeout=None):
        self._ensure_running()
        return self.nav_fusion.wait_for_update(timeout=timeout)

    def get_data_json(self):
        self._ensure_running()
        fusion_data = self.nav_fusion.get_latest()
        return fusion_data.to_json()

    def get_data_binary(self):
        self._ensure_running()
        fusion_data = self.nav_fusion.get_latest()
        return fusion_data.to_bytes() 

    # API pro zracování ODM messages
    def process_odm_message(self, odm_message):
        self._ensure_running()
        # parse "ODM<mono>,<omega>,<alfa>,<speed>"
        payload = odm_message[3:]
        fields = payload.split(",")
        # konvert si hodnoty
        mono = int(fields[0], 10)    
        omega = int(fields[1], 10)
        angle = int(fields[2], 10)
        speed = int(fields[3], 10)
        # update nav fusion
        self.nav_fusion.on_odm_data(mono, omega, angle, speed)
        # update gnss device odometry
        ubx = build_odm(mono,speed)
        self.gnss.send_ubx(ubx)


