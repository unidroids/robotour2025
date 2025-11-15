"""dispatcher.py – jednovláknový router příchozích zpráv na handlery

Běží ve vlastním vlákně, čte z RX FIFO (HoverboardSerial.get_message).
Handlery jsou registrovány podle kódu zprávy (první 3 ASCII znaky payloadu).
Věta začíná '$' následuje 3znakový kód, pak data, pak '*CS' a CRLF.

Design cíle:
  - Time‑critical: minimální režie, žádné I/O v hot‑path
  - Bezpečnost: výjimka v handleru nezabije vlákno, jen se zapíše do počitadel
  - Rozšiřitelnost: jednoduché API pro registraci handlerů + default handler

Použití (z service.py):
  disp = MessageDispatcher(wrap_serial)
  disp.register_handler('IAM', handlers.AckHandler)
  disp.register_handler('INM', handlers.NackHandler)
  disp.set_default_handler(handlers.DummyHandler)
  ...
  disp.start()

Handler je třída, která má metodu handle(msg: bytes) -> None.

Pozn.: Tohle je jen "router" – vlastní logika je v handlerech.
"""

__all__ = [
    "MessageDispatcher"
]

# dispatcher.py
import threading
from typing import Callable, Dict
from wrap_serial import WrapSerial
class MessageDispatcher:
    """Jednovláknový dispatcher příchozích zpráv na registrované handlery."""

    def __init__(self, wrap_serial:WrapSerial):
        self.wrap_serial = wrap_serial
        self._stop_event = threading.Event()
        self._thread = None 
        self._handlers: Dict[str, object] = {}  
        self._messages_handled = 0
        self._messages_unknown = 0
        self._messages_errors = 0
        self._messages_ignored = 0

    def register_handler(self, typ: str, handler_obj: object):
        if not hasattr(handler_obj, "handle") or not callable(getattr(handler_obj, "handle")):
            raise TypeError("Handler must implement handle(message_bytes)")
        self._handlers[typ.upper()] = handler_obj

    def start(self):
        if self._thread and self._thread.is_alive():
            return  # už běží        
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        t = self._thread
        if t and t.is_alive():
            t.join(timeout=0.5)
        self._thread = None        

    def stats(self):
        return (self._messages_handled,
                self._messages_unknown,
                self._messages_errors,
                self._messages_ignored)

    def _run(self):
        while not self._stop_event.is_set():
            try:
                message = self.wrap_serial.get_message(timeout=1.0)
                # kontrola zpravy, identifikace hlavičky a předání handlerovi
                if not message:
                    continue
                if len(message) < 10 or message[0] != ord('#'):
                    self._messages_ignored += 1
                    continue
                # find first coma [,]
                coma_idx = message.index(b',')
                typ = message[1:coma_idx].decode('ascii', errors='ignore').upper()
                handler = self._handlers.get(typ)
                if handler:
                    handler.handle(message)
                    self._messages_handled += 1
                else:
                    print(f"[MessageDispatcher] unhandled message {typ} ({len(message)} bytes)")
                    self._messages_unknown += 1
                    
            except Exception as e:
                print(f"[MessageDispatcher] Error: {e}")
                self._messages_errors += 1

# DEMO
if __name__ == '__main__':
    from wrap_serial import WrapSerial
    import time

    class DummyHandler:
        def __init__(self, every=10):
            self.count = 0
            self.every = every

        def handle(self, message_bytes: bytes):
            self.count += 1
            if self.count % self.every == 0:
                print(f"#{self.count}:", message_bytes.decode(errors='ignore').strip())

    wrap = WrapSerial()
    handler = DummyHandler()
    md = MessageDispatcher(wrap)
    wrap.start()
    md.register_handler('UNIHEADINGA', DummyHandler(10))
    md.start()


    try:
        while True:
            time.sleep(10)
    except KeyboardInterrupt:
        pass
    finally:
        wrap.stop()
        md.stop()
        