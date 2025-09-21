# ubx_dispatcher.py

import threading
import sys

class UbxDispatcher:
    def __init__(self, gnss_serial):
        self.gnss_serial = gnss_serial
        self.handlers = {}  # {(msg_class, msg_id): handler_instance}
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def register_handler(self, msg_class, msg_id, handler):
        self.handlers[(msg_class, msg_id)] = handler

    def start(self):
        self._stop_event.clear()
        self._thread.start()

    def stop(self):
        self._stop_event.set()

    def _run(self):
        while not self._stop_event.is_set():
            msg = self.gnss_serial.get_ubx_message(timeout=1.0)
            if msg:
                msg_class, msg_id, payload = msg
                handler = self.handlers.get((msg_class, msg_id))
                if handler:
                    try:
                        handler.handle(msg_class, msg_id, payload)
                    except Exception as e:
                        print(f"[Dispatcher] Handler error for 0x{msg_class:02X} 0x{msg_id:02X}: {e}", file=sys.stderr)
                else:
                    print(f"[Dispatcher] unhandled UBX 0x{msg_class:02X} 0x{msg_id:02X} ({len(payload)} bytes)")

# ------------- DEMO / TEST -------------
if __name__ == '__main__':
    from gnss_serial import GnssSerialIO
    import time
    
    # Demo handler
    class PrintHandler:
        def handle(self, msg_class, msg_id, payload):
            print(f"Handled UBX 0x{msg_class:02X} 0x{msg_id:02X} len={len(payload)}")

    gnss = GnssSerialIO('/dev/gnss1')
    gnss.open()

    disp = UbxDispatcher(gnss)
    # Registrace pouze NAV-HPPOSLLH (0x01, 0x14)
    disp.register_handler(0x01, 0x14, PrintHandler())

    disp.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Bye")
    finally:
        disp.stop()
        gnss.close()
