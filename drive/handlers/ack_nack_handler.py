from .h_utils import parse_message
from typing import Callable

__all__ = [
    "AckNackHandler",
]


class AckNackHandler:
    def __init__(self, callback: Callable[[int, int, int, int, int, int, int], None]):
        self._callback = callback
        self.count = 0
        self._last_message = None

    def handle(self, message_bytes: bytes):
        # parsování zprávy
        code, fields = parse_message(message_bytes)
        print(f"Handling message code: {code} with fields: {fields}")
        self.count += 1
        if code == 'IAM':
            if len(fields) != 5:
                print(f"[AckNackHandler] Invalid IAM message length: {len(fields)}, expected 5 {fields}")
                return
            cmd = int(fields[0], 10)
            p1 = int(fields[1], 10)
            p2 = int(fields[2], 10)
            p3 = int(fields[3], 10)
            p4 = int(fields[4], 10)
            self._callback(cmd, p1, p2, p3, p4, 0, 0)
        elif code == 'INM':
            if len(fields) != 7:
                print(f"[AckNackHandler] Invalid INM message length: {len(fields)}, expected 7 {fields}")
                return
            cmd = int(fields[0], 10)
            p1 = int(fields[1], 10)
            p2 = int(fields[2], 10)
            p3 = int(fields[3], 10)
            p4 = int(fields[4], 10)
            ie = int(fields[5], 10)
            ce = int(fields[6], 10)
            self._callback(cmd, p1, p2, p3, p4, ie, ce)
        else:
            print(f"[AckNackHandler] Unknown message code: {code}")