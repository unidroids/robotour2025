# ubx_decode.py
# -*- coding: utf-8 -*-
import struct

def decode_mon_sys(payload: bytes) -> dict:
    """
    UBX-MON-SYS (0x0A 0x39), běžná délka 24 B (msgVer=1).
    Vrací hlavní ukazatele: cpu/mem/io usage, runtime, počty notice/warn/err, teplotu.
    """
    if len(payload) < 19:
        return {"len": len(payload), "raw_hex": payload.hex()}
    # <BBBBBBBB = msgVer, bootType, cpuLoad, cpuLoadMax, memUsage, memUsageMax, ioUsage, ioUsageMax
    msgVer, bootType, cpuLoad, cpuLoadMax, memUsage, memUsageMax, ioUsage, ioUsageMax = struct.unpack_from(
        "<BBBBBBBB", payload, 0
    )
    runTime = struct.unpack_from("<I", payload, 8)[0]
    notice, warn, err = struct.unpack_from("<HHH", payload, 12)
    temp = struct.unpack_from("<b", payload, 18)[0]
    return {
        "msgVer": msgVer,
        "bootType": bootType,
        "cpuLoad%": cpuLoad,
        "cpuLoadMax%": cpuLoadMax,
        "memUsage%": memUsage,
        "memUsageMax%": memUsageMax,
        "ioUsage%": ioUsage,
        "ioUsageMax%": ioUsageMax,
        "runTime_s": runTime,
        "notice": notice,
        "warn": warn,
        "err": err,
        "temp_C": temp,
    }


def decode_mon_txbuf(payload: bytes) -> dict:
    """
    UBX-MON-TXBUF (0x0A 0x08), délka 28 B.
    pending[6] (U2), usage[6] % (U1), peak[6] % (U1), totalUsage (U1), totalPeak (U1), errors (U1).
    """
    if len(payload) < 28:
        return {"len": len(payload), "raw_hex": payload.hex()}
    pending = list(struct.unpack_from("<6H", payload, 0))
    usage   = list(struct.unpack_from("<6B", payload, 12))
    peak    = list(struct.unpack_from("<6B", payload, 18))
    tUsage, tPeak, errors = struct.unpack_from("<BBB", payload, 24)
    return {
        "pending": pending,
        "usage%": usage,
        "peak%": peak,
        "totalUsage%": tUsage,
        "totalPeak%": tPeak,
        "errors": errors,
    }
