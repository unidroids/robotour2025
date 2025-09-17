# itow_extract.py
import struct

CLASS_NAV = 0x01
CLASS_ESF = 0x10

ID_NAV_ATT      = 0x05
ID_NAV_VELNED   = 0x12
ID_NAV_HPPOSLLH = 0x14  # iTOW @ +4 (byte0=version)
ID_NAV_EOE      = 0x61
ID_ESF_INS      = 0x15  # iTOW @ +8 (viz HPS spec)

NAV_ATT      = (CLASS_NAV, ID_NAV_ATT)
NAV_VELNED   = (CLASS_NAV, ID_NAV_VELNED)
NAV_HPPOSLLH = (CLASS_NAV, ID_NAV_HPPOSLLH)
NAV_EOE      = (CLASS_NAV, ID_NAV_EOE)
ESF_INS      = (CLASS_ESF, ID_ESF_INS)

def extract_itow(cls_id, payload: bytes):
    if cls_id == NAV_HPPOSLLH:
        return struct.unpack_from("<I", payload, 4)[0] if len(payload) >= 8 else None
    elif cls_id == ESF_INS:
        return struct.unpack_from("<I", payload, 8)[0] if len(payload) >= 12 else None
    elif cls_id in (NAV_ATT, NAV_VELNED, NAV_EOE):
        return struct.unpack_from("<I", payload, 0)[0] if len(payload) >= 4 else None
    return None
