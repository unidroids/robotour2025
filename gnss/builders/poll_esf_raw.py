import struct
def build_poll_esf_raw():
    sync1, sync2 = 0xb5, 0x62
    cls, id = 0x10, 0x03
    length = 0
    payload = b''
    # spočítat checksum podle UBX (pro prázdný payload je snadné)
    ck_a, ck_b = 0, 0
    for b in [cls, id, 0, 0]:  # header + length (little-endian)
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return struct.pack('<BBBBBB', sync1, sync2, cls, id, 0, 0) + bytes([ck_a, ck_b])
