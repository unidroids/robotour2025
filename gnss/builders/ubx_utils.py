import struct

def ubx_checksum(data: bytes) -> bytes:
    """Returns 2-byte UBX checksum (CK_A, CK_B) for input data (class+id+len+payload)."""
    ck_a, ck_b = 0, 0
    for b in data:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return struct.pack('<BB', ck_a, ck_b)

def ubx_packet(msg_class: int, msg_id: int, payload: bytes = b'') -> bytes:
    """Builds full UBX message (including header, checksum). Returns bytes starting with b'\xB5\x62'."""
    length = len(payload)
    # Header: sync chars + class + id + length (little endian)
    header = struct.pack('<BBBBH', 0xB5, 0x62, msg_class, msg_id, length)
    core = struct.pack('<BBH', msg_class, msg_id, length) + payload
    chksum = ubx_checksum(core)
    return header + payload + chksum
