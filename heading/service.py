# service.py

import threading
from typing import Any, Dict

from wrap_serial import WrapSerial
from dispatcher import MessageDispatcher
from handlers.uniheadinga_handler import UniHeadinAHandler
import time

class UnicoreService:

    def __init__(self):
        self.running = False
        self._initialized = False
        self._lock = threading.Lock()

        self.unihaedinga_handler = None
        self.wrap_serial = None
        self.dispatcher = None

        self._start_at = 0.0

    def start(self):
        with self._lock:
            if self.running:
                return "ALREADY_RUNNING"
            if not self._initialized:
                self.unihaedinga_handler = UniHeadinAHandler()
                self.wrap_serial = WrapSerial()
                self.dispatcher = MessageDispatcher(self.wrap_serial)
                self.dispatcher.register_handler("UNIHEADINGA",self.unihaedinga_handler)
                self._initialized = True
            self.dispatcher.start()
            self.wrap_serial.start()
            self.running = True
            self._start_at = time.monotonic()
            print("[SERVICE] STARTED")
            return "OK"

    def stop(self):
        with self._lock:
            if not self.running:
                return "NOT_RUNNING"
            if self.wrap_serial:
                self.wrap_serial.stop()
            if self.dispatcher:
                self.dispatcher.stop()
            self.running = False
            self.unihaedinga_handler = None
            self.wrap_serial = None
            self.dispatcher = None
            self._initialized = False
            print("[SERVICE] STOPPED")
            return "OK"

    def _ensure_running(self):
        if not self.running or not self._initialized:
            raise RuntimeError("[GNSS SERVICE] Service is not running. Call START first.")

    # API pro diagnostiku
    def get_state(self) -> Dict[str, Any]:
        with self._lock:
            running = self.running
            started_at = self._start_at

        if not running:
            return {
                "service": "HEADING",
                "status": "RUNNING" if running else "STOPPED"
            }

        # načti stats
        (
            open_failures,
            rx_bytes,
            tx_bytes,
            rx_msgs,
            tx_frames,
            rx_over,
            tx_over,
            junk_count,
            bad_char_count,
            crc32_error_count,
            too_long_count,
            sentences_parsed,
        ) = self.wrap_serial.stats()

        (   dispatcher_handled,
            dispatcher_unknown,
            dispatcher_errors,
            dispatcher_ignored,
        ) = self.dispatcher.stats()

        return {
            "service": "HEADING",
            "status": "RUNNING" if running else "STOPPED",
            "started_at_mono": started_at,
            "serial": {
                "device": self.wrap_serial.cfg.device,
                "baud": self.wrap_serial.cfg.baudrate,
                "open_failures": open_failures,
                "rx_bytes": rx_bytes,
                "tx_bytes": tx_bytes,
                "rx_msgs": rx_msgs,
                "tx_frames": tx_frames,
                "rx_overflows": rx_over,
                "tx_overflows": tx_over,
                "parser": {
                    "junk": junk_count,
                    "bad_char": bad_char_count,
                    "crc_error": crc32_error_count,
                    "too_long": too_long_count,
                    "sentences_parsed": sentences_parsed,
                },
            },
            "dispatcher": {
                "handled": dispatcher_handled,
                "unknown": dispatcher_unknown,
                "errors": dispatcher_errors,
                "ignored": dispatcher_ignored,
            },  
        }

    # API pro získání posledního headingu
    def get_heading(self):
        self._ensure_running()
        return self.unihaedinga_handler.get_lastest()



# DEMO
if __name__ == '__main__':
    service = UnicoreService()
    print(service.get_state())
    service.start()
    time.sleep(0.5)
    print(service.get_heading())
    time.sleep(0.5)
    print(service.get_state())
    service.stop()
