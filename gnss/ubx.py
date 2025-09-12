# ubx.py – UBX protokol pro Robotour GNSS

SYNC1 = 0xB5
SYNC2 = 0x62

def checksum(data: bytes) -> bytes:
    """Fletcher checksum (CK_A, CK_B)"""
    ck_a = 0
    ck_b = 0
    for b in data:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return bytes([ck_a, ck_b])

def build_msg(msg_class: int, msg_id: int, payload: bytes = b"") -> bytes:
    """Sestaví UBX zprávu včetně checksum"""
    length = len(payload).to_bytes(2, "little")
    head = bytes([SYNC1, SYNC2, msg_class, msg_id]) + length + payload
    ck = checksum(head[2:])  # od CLASS po konec payloadu
    return head + ck

def parse_stream(buf: bytes):
    """Najde a vrátí první kompletní UBX zprávu"""
    start = buf.find(b"\xB5\x62")
    if start == -1 or len(buf) - start < 8:
        return None, None, None, buf
    msg_class = buf[start+2]
    msg_id    = buf[start+3]
    length    = buf[start+4] | (buf[start+5] << 8)
    end = start + 6 + length + 2
    if len(buf) < end:
        return None, None, None, buf[start:]
    payload = buf[start+6:end-2]
    ck = buf[end-2:end]
    if checksum(buf[start+2:end-2]) != ck:
        # CRC fail, posuň buffer o 2
        return None, None, None, buf[start+2:]
    return msg_class, msg_id, payload, buf[end:]

def parse_nav_pvt(payload: bytes):
    """Dekóduje NAV-PVT zprávu (class=0x01, id=0x07)"""
    if len(payload) < 92:
        return None
    iTOW = int.from_bytes(payload[0:4], "little")
    lat  = int.from_bytes(payload[28:32], "little", signed=True) / 1e7
    lon  = int.from_bytes(payload[24:28], "little", signed=True) / 1e7
    alt  = int.from_bytes(payload[36:40], "little", signed=True) / 1000.0
    hAcc = int.from_bytes(payload[40:44], "little")
    vAcc = int.from_bytes(payload[44:48], "little")
    speed= int.from_bytes(payload[60:64], "little", signed=True) / 1000.0
    head = int.from_bytes(payload[84:88], "little", signed=True) / 1e5
    numSV= payload[23]
    fixType = payload[20]
    return {
        "time": iTOW,
        "lat": lat,
        "lon": lon,
        "alt": alt,
        "speed": speed,
        "heading": head,
        "numSV": numSV,
        "fixType": fixType,
        "hAcc": hAcc,
        "vAcc": vAcc
    }


def build_esf_meas_ticks(time_tag: int,
                         left_ticks: int, left_dir: int,
                         right_ticks: int, right_dir: int):
    """
    UBX-ESF-MEAS zpráva se dvěma měřeními – wheel ticks left/right.
    - time_tag: U4 (např. iTOW nebo systémový ms)
    - left_ticks, right_ticks: unsigned 32-bit (budou oříznuty na 23 bitů)
    - left_dir, right_dir: 0 = forward, 1 = backward
    """

    payload = b""

    # --- header části payloadu ---
    payload += (time_tag & 0xFFFFFFFF).to_bytes(4, "little")   # U4 timeTag
    payload += (0).to_bytes(2, "little")                      # X2 flags (vše 0)
    payload += (0).to_bytes(2, "little")                      # U2 id (0 = host SW)

    # pomocná funkce na zabalení jednoho měření
    def pack_ticks(ticks: int, direction: int, dtype: int) -> bytes:
        # dataField = spodních 23 bitů + dir bit v 23. pozici
        data_field = (ticks & 0x7FFFFF) | ((direction & 0x1) << 23)
        # celé pole (32 bitů): dataField (24 b) + dataType (6 b)
        word = (data_field & 0xFFFFFF) | ((dtype & 0x3F) << 24)
        return word.to_bytes(4, "little")

    # left wheel → dataType = 8
    payload += pack_ticks(left_ticks, left_dir, 8)
    # right wheel → dataType = 9
    payload += pack_ticks(right_ticks, right_dir, 9)

    # vrátí kompletní UBX zprávu
    return build_msg(0x10, 0x02, payload)
