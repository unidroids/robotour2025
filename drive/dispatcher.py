"""dispatcher.py – jednovláknový router příchozích zpráv na handlery

Běží ve vlastním vlákně, čte z RX FIFO (HoverboardSerial.get_message).
Handlery jsou registrovány podle kódu zprávy (první 3 ASCII znaky payloadu).
Věta začíná '$' následuje 3znakový kód, pak data, pak '*CS' a CRLF.

Design cíle:
  - Time‑critical: minimální režie, žádné I/O v hot‑path
  - Bezpečnost: výjimka v handleru nezabije vlákno, jen se zapíše do počitadel
  - Rozšiřitelnost: jednoduché API pro registraci handlerů + default handler

Použití (z service.py):
  disp = MessageDispatcher(hb_serial)
  disp.register_handler('IAM', handlers.AckHandler)
  disp.register_handler('INM', handlers.NackHandler)
  disp.set_default_handler(handlers.DummyHandler)
  ...
  disp.start()

Handler je třída, která má metodu handle(msg: bytes) -> None.

Pozn.: Tohle je jen "router" – vlastní logika je v handlerech.
"""
# dispatcher.py
import threading
from typing import Callable, Dict

class MessageDispatcher:
    """Jednovláknový dispatcher příchozích zpráv na registrované handlery."""

    def __init__(self, hb_serial):
        self.hb_serial = hb_serial
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._handlers: Dict[str, object] = {}  # 'GGA' -> handler

    def register_handler(self, typ: str, handler_obj: object):
        if not hasattr(handler_obj, "handle") or not callable(getattr(handler_obj, "handle")):
            raise TypeError("Handler must implement handle(message_bytes)")
        self._handlers[typ.upper()] = handler_obj

    def start(self):
        self._stop_event.clear()
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join(timeout=0.5)

    def _run(self):
        while not self._stop_event.is_set():
            try:
                message = self.hb_serial.get_message(timeout=1.0)
                if not message:
                    continue
                # raw bytes → hlavička je $ + 3 znaky
                if len(message) < 4 or message[0] != ord('$'):
                    continue
                typ = message[1:4].decode('ascii', errors='ignore').upper()
                handler = self._handlers.get(typ)
                if handler:
                    handler.handle(message)
                else:
                    print(f"[MessageDispatcher] unhandled message {typ} ({len(message)} bytes)")
                    
            except Exception as e:
                print(f"[MessageDispatcher] Error: {e}")

# DEMO
if __name__ == '__main__':
    from hb_serial import HoverboardSerial
    import time

    class PrintIAM:
        def __init__(self, every=10):
            self.count = 0
            self.every = every

        def handle(self, message_bytes: bytes):
            self.count += 1
            if self.count % self.every == 0:
                print(f"[ODM]#{self.count}:", message_bytes.decode(errors='ignore').strip())

    hb = HoverboardSerial()
    hb.start()
    handler = PrintIAM()
    md = MessageDispatcher(hb)
    md.register_handler('ODM', handler)
    md.start()


    try:
        while True:
            time.sleep(10)
    except KeyboardInterrupt:
        pass
    finally:
        md.stop()
        hb.stop()
        