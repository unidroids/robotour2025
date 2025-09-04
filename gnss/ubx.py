# ubx.py
# Minimalni implementace pro UBX protokol

import struct

SYNC1 = 0xB5
SYNC2 = 0x62

def checksum(payload: bytes):
    ck_a = 0
    ck_b = 0
    for b in payload:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b

def build_msg(cls: int, id_: int, payload: bytes = b""):
    length = len(payload)
    header = struct.pack("<BBBBH", SYNC1, SYNC2, cls, id_, length)
    ck_a, ck_b = checksum(bytes([cls, id_, length & 0xFF, length >> 8]) + payload)
    return header + payload + bytes([ck_a, ck_b])
