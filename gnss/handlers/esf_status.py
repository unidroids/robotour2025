import struct

class EsfStatusHandler:
    def __init__(self):
        self.count = 0

    def handle(self, msg_class, msg_id, payload):
        self.count += 1
        if len(payload) < 16:
            print(f"[ESF-STATUS] Too short ({len(payload)} bytes): {payload.hex()}")
            return
        try:
            # Hlavička 16B
            iTOW, version, initStatus1, initStatus2 = struct.unpack('<IBBB', payload[:7])
            reserved0 = payload[7:12]
            fusionMode = payload[12]
            reserved1 = payload[13:15]
            numSens = payload[15]
            print(f"[ESF-STATUS] iTOW={iTOW} ver={version} fusionMode={fusionMode} numSens={numSens}")

            # Opakovaná sekce: každý senzor 4B
            for i in range(numSens):
                offset = 16 + i*4
                if len(payload) < offset + 4:
                    print(f"[ESF-STATUS] Sensor {i} data incomplete: {payload[offset:].hex()}")
                    continue
                sensStatus1 = payload[offset]
                sensStatus2 = payload[offset+1]
                freq = payload[offset+2]
                faults = payload[offset+3]
                sensor_type = sensStatus1 & 0x3F
                used = (sensStatus1 >> 6) & 0x01
                ready = (sensStatus1 >> 7) & 0x01
                calibStatus = sensStatus2 & 0x03
                timeStatus = (sensStatus2 >> 2) & 0x03
                print(f"    Sensor {i}: type={sensor_type} used={used} ready={ready} calibStatus={calibStatus} "
                      f"timeStatus={timeStatus} freq={freq}Hz faults=0x{faults:02X}")
        except Exception as e:
            print(f"[ESF-STATUS] Handler error: {e} | Payload: {payload.hex()}")
