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


# ---------------------- ESF-MEAS ----------------------

def build_esf_meas_speed(speed_mps: float, time_tag: int = 0):
    """
    UBX-ESF-MEAS s jedním měřením – vehicle speed
    speed_mps ... rychlost v m/s
    """
    speed_mm = int(speed_mps * 1000)  # mm/s
    payload = b""
    payload += time_tag.to_bytes(4, "little")  # timeTag
    payload += (0).to_bytes(1, "little")       # flags
    payload += (0).to_bytes(1, "little")       # id
    payload += (1).to_bytes(2, "little")       # dataCount
    payload += speed_mm.to_bytes(4, "little", signed=True)  # data
    payload += (0x20).to_bytes(2, "little")    # dataType = vehicle speed
    return build_msg(0x10, 0x02, payload)

def build_esf_meas_ticks(left_ticks: int, right_ticks: int, time_tag: int = 0):
    """
    UBX-ESF-MEAS se dvěma měřeními – wheel ticks left/right
    Tick counts = kumulativní čítač od začátku (signed 32b)
    """
    payload = b""
    payload += time_tag.to_bytes(4, "little")  # timeTag
    payload += (0).to_bytes(1, "little")       # flags
    payload += (0).to_bytes(1, "little")       # id
    payload += (2).to_bytes(2, "little")       # dataCount

    # left wheel
    payload += left_ticks.to_bytes(4, "little", signed=True)
    payload += (0x00).to_bytes(2, "little")    # dataType = wheel tick sensor 0

    # right wheel
    payload += right_ticks.to_bytes(4, "little", signed=True)
    payload += (0x01).to_bytes(2, "little")    # dataType = wheel tick sensor 1

    return build_msg(0x10, 0x02, payload)


    