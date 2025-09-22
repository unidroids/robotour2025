# handlers/mon_comms.py

import struct

class MonCommsHandler:
    """Handler for UBX-MON-COMMS (0x0a 0x36)"""
    def handle(self, msg_class, msg_id, payload):
        if len(payload) < 8:
            print("[MON-COMMS] Wrong payload length:", len(payload))
            return
        version = payload[0]
        nPorts = payload[1]
        txErrors = payload[2]
        print(f"[MON-COMMS] version={version} nPorts={nPorts} txErrors=0x{txErrors:02X}")
