from h_utils import parse_message

__all__ = [
    "AckNackHandler",
]


class AckNackHandler:
    def __init__(self, callback: Callable[(cmd,p1,p2,p3,p4,ie,ce), None]):
        self._callback = callback
        self.count = 0
        self._last_message = None

    def handle(self, message_bytes: bytes):
        # parsování zprávy
        code, fields = parse_message(message_bytes)
        print(f"Handling message code: {code} with fields: {fields}")
        self.count += 1
        #if self.count % self.every == 0:
        #    print(f"#{self.count}:", message_bytes.decode(errors='ignore').strip())