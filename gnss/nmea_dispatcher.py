# nmea_dispatcher.py
import threading
from typing import Callable, Dict

class NmeaDispatcher:
    """
    Čte validované NMEA věty (bytes včetně CRLF) z GnssSerialIO.get_nmea_sentence()
    a směruje je na handlery dle 5znakové hlavičky (např. 'GPGGA') nebo typu ('GGA').
    Handler je objekt s metodou:
        handle(sentence_bytes: bytes) -> None
    """

    def __init__(self, gnss_serial):
        self.gnss_serial = gnss_serial
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._handlers5: Dict[str, object] = {}  # 'GPGGA' -> handler
        self._handlers3: Dict[str, object] = {}  # 'GGA' -> handler

    def register_5(self, header5: str, handler_obj: object):
        if not hasattr(handler_obj, "handle") or not callable(getattr(handler_obj, "handle")):
            raise TypeError("Handler must implement handle(sentence_bytes)")
        self._handlers5[header5.upper()] = handler_obj

    def register_3(self, typ3: str, handler_obj: object):
        if not hasattr(handler_obj, "handle") or not callable(getattr(handler_obj, "handle")):
            raise TypeError("Handler must implement handle(sentence_bytes)")
        self._handlers3[typ3.upper()] = handler_obj

    def start(self):
        self._stop_event.clear()
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join(timeout=0.5)

    def _run(self):
        while not self._stop_event.is_set():
            try:
                sent = self.gnss_serial.get_nmea_sentence(timeout=1.0)
                if not sent:
                    continue
                # raw bytes → hlavička je $ + 5 znaků
                if len(sent) < 7 or sent[0] != ord('$'):
                    continue
                head5 = sent[1:6].decode('ascii', errors='ignore').upper()
                typ3  = head5[-3:]
                handler = self._handlers5.get(head5) or self._handlers3.get(typ3)
                if handler:
                    handler.handle(sent)
                else:
                    print(f"[NmeaDispatcher] unhandled NMEA {head5} ({len(sent)} bytes)")
                    
            except Exception as e:
                print(f"[NmeaDispatcher] Error: {e}", file=sys.stderr)

# DEMO
if __name__ == '__main__':
    from gnss_serial import GnssSerialIO
    import time

    class PrintGGA:
        def __init__(self, every=10):
            self.count = 0
            self.every = every

        def handle(self, sentence_bytes: bytes):
            self.count += 1
            if self.count % self.every == 0:
                print("[GGA]", sentence_bytes.decode(errors='ignore').strip())

    gnss = GnssSerialIO('/dev/gnss1')
    gnss.open()

    from ubx_dispatcher import UbxDispatcher
    ubx_disp = UbxDispatcher(gnss)
    ubx_disp.start()

    nd = NmeaDispatcher(gnss)
    nd.register_3('GGA', PrintGGA())
    nd.start()


    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        nd.stop()
        gnss.close()
        