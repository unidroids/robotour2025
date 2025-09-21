# handlers/mon_sys.py

import struct
import time

class MonSysHandler:
    """Handler for UBX-MON-SYS (0x0a 0x39)"""
    def handle(self, msg_class, msg_id, payload):
        if len(payload) != 24:
            print("[MON-SYS] Wrong payload length:", len(payload))
            return
        (
            msgVer,
            bootType,
            cpuLoad, cpuLoadMax,
            memUsage, memUsageMax,
            ioUsage, ioUsageMax,
            runTime,
            noticeCount,
            warnCount,
            errorCount,
            tempValue,
            _  # reserved
        ) = struct.unpack('<BB BB BB BB I H H H b 5s', payload)

        print(f"[MON-SYS] ver={msgVer} boot={bootType} cpu={cpuLoad}%/{cpuLoadMax}% mem={memUsage}%/{memUsageMax}% io={ioUsage}%/{ioUsageMax}% t={tempValue}°C errors={errorCount} warnings={warnCount} notices={noticeCount} up={runTime}s")

