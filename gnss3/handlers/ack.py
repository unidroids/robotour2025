import struct

class AckHandler:
    def __init__(self):
        self.count_ack = 0
        self.count_nak = 0

    def handle(self, msg_class, msg_id, payload):
        if len(payload) != 2:
            print(f"[ACK] Wrong payload length: {len(payload)} bytes")
            return
        clsID, msgID = struct.unpack('<BB', payload)
        if msg_id == 0x01:
            self.count_ack += 1
            print(f"[ACK] ACK for msg_class=0x{clsID:02X} msg_id=0x{msgID:02X} (total ACK: {self.count_ack})")
        elif msg_id == 0x00:
            self.count_nak += 1
            print(f"[ACK] NAK for msg_class=0x{clsID:02X} msg_id=0x{msgID:02X} (total NAK: {self.count_nak})")
        else:
            print(f"[ACK] Unknown ACK type: msg_id=0x{msg_id:02X}")

