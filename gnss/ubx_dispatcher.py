# ubx_dispatcher.py
import threading
from typing import Dict, Tuple

class UbxDispatcher:
    """
    Čte raw UBX rámce z GnssSerialIO.get_ubx_frame() a předává je handlerům.
    Předpoklad: rámce už jsou validované (sync, délka, checksum) v GnssSerialIO.
    Handler je objekt s metodou:
        handle(msg_class: int, msg_id: int, payload: bytes) -> None
    """

    def __init__(self, gnss_serial):
        self.gnss_serial = gnss_serial
        self.handlers: Dict[Tuple[int, int], object] = {}
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def register_handler(self, msg_class: int, msg_id: int, handler_obj: object):
        if not hasattr(handler_obj, "handle") or not callable(getattr(handler_obj, "handle")):
            raise TypeError("Handler must implement handle(msg_class, msg_id, payload)")
        self.handlers[(msg_class, msg_id)] = handler_obj

    def start(self):
        self._stop_event.clear()
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join(timeout=0.5)

    def _run(self):
        get_frame = self.gnss_serial.get_ubx_frame
        while not self._stop_event.is_set():
            frm = get_frame(timeout=1.0)
            if not frm:
                continue
            # frm: [B5 62 class id len_lo len_hi payload... ckA ckB]
            msg_class = frm[2]
            msg_id    = frm[3]
            length    = frm[4] | (frm[5] << 8)
            payload   = frm[6:6+length]

            handler = self.handlers.get((msg_class, msg_id))
            if handler:
                handler.handle(msg_class, msg_id, payload)
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
    #disp.register_handler(0x01, 0x14, PrintHandler())
    disp.register_handler(0x01, 0x17, PrintHandler())

    disp.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Bye")
    finally:
        disp.stop()
        gnss.close()
