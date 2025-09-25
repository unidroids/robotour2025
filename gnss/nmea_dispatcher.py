# nmea_dispatcher.py
import threading
import sys
from typing import Callable, Dict, Optional

class NmeaDispatcher:
    """
    Tahá z gnss_serial get_nmea_sentence() (raw bytes s CRLF) a volá handlery.
    Registrace podle:
      - 5 znakové hlavičky (např. 'GPGGA'), nebo
      - 3 znakového typu (např. 'GGA') – platí pro libovolný talker.
    """

    def __init__(self, gnss_serial, decode_ascii: bool = False):
        self.gnss_serial = gnss_serial
        self.decode_ascii = decode_ascii
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._handlers5: Dict[str, Callable[[bytes], None]] = {}
        self._handlers3: Dict[str, Callable[[bytes], None]] = {}

    def register_5(self, header5: str, handler: Callable[[bytes], None]):
        """Např. 'GPGGA'."""
        self._handlers5[header5.upper()] = handler

    def register_3(self, typ3: str, handler: Callable[[bytes], None]):
        """Např. 'GGA'."""
        self._handlers3[typ3.upper()] = handler

    def start(self):
        self._stop_event.clear()
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join(timeout=0.5)

    def _run(self):
        while not self._stop_event.is_set():
            sent = self.gnss_serial.get_nmea_sentence(timeout=1.0)
            if not sent:
                continue
            try:
                # raw bytes → hlavička je $ + 5 znaků
                if len(sent) < 7 or sent[0] != ord('$'):
                    continue
                head5 = sent[1:6].decode('ascii', errors='ignore').upper()
                typ3  = head5[-3:]

                h = self._handlers5.get(head5) or self._handlers3.get(typ3)
                if h:
                    # předáváme raw bytes; pokud chceš text, handler si sám dekóduje
                    h(sent)
                else:
                    # volitelné: tichý ignore
                    # if self.decode_ascii: print(sent.decode(errors='ignore').strip())
                    pass
            except Exception as e:
                print(f"[NmeaDispatcher] Error: {e}", file=sys.stderr)

# DEMO
if __name__ == '__main__':
    from gnss_serial import GnssSerialIO
    import time

    def print_gga(sent_bytes: bytes):
        print("NMEA GGA:", sent_bytes.decode(errors='ignore').strip())

    gnss = GnssSerialIO('/dev/gnss1')
    gnss.open()

    nd = NmeaDispatcher(gnss)
    nd.register_3('GGA', print_gga)
    nd.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        nd.stop()
        gnss.close()
        