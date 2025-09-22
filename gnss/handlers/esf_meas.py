import struct

class EsfMeasHandler:
    def __init__(self):
        self.count = 0

    def handle(self, msg_class, msg_id, payload):
        self.count += 1
        if len(payload) < 8:
            print(f"[ESF-MEAS] Payload too short: {len(payload)} bytes")
            return

        # Rozparsuj header
        timeTag, flags, provider_id = struct.unpack('<I H H', payload[:8])
        numMeas = (flags >> 11) & 0x1F
        timeMarkSent = flags & 0x03
        timeMarkEdge = (flags >> 2) & 0x01
        calibTtagValid = (flags >> 3) & 0x01

        print(f"[ESF-MEAS] count={self.count} timeTag={timeTag} provider_id=0x{provider_id:04X} numMeas={numMeas} "
              f"timeMarkSent={timeMarkSent} timeMarkEdge={timeMarkEdge} calibTtagValid={calibTtagValid}")

        # Každé měření je 4 byty
        for i in range(numMeas):
            offset = 8 + i*4
            if len(payload) < offset+4:
                print(f"    [ESF-MEAS] Measurement {i} truncated")
                continue
            raw = payload[offset:offset+4]
            data, = struct.unpack('<I', raw)
            dataType = (data >> 24) & 0x3F
            value = data & 0xFFFFFF  # lower 24 bits
            # pro typy 8,9,10: bit 23 = direction
            direction = (value >> 23) & 0x01
            ticks = value & 0x7FFFFF
            if dataType in (8, 9, 10):
                print(f"    [ESF-MEAS] type={dataType} ticks={ticks} dir={direction} raw=0x{value:06X}")
            elif dataType == 11:
                # speed, signed 24bit
                if value & 0x800000:
                    speed = value - 0x1000000
                else:
                    speed = value
                print(f"    [ESF-MEAS] type=11 speed={speed*1e-3:.3f} m/s (raw={speed})")
            else:
                print(f"    [ESF-MEAS] type={dataType} value=0x{value:06X} (raw=0x{data:08X})")

        # Optional calibTtag na konci (pouze pokud calibTtagValid==1)
        calib_ttag_offset = 8 + numMeas*4
        if calibTtagValid and len(payload) >= calib_ttag_offset + 4:
            calibTtag, = struct.unpack('<I', payload[calib_ttag_offset:calib_ttag_offset+4])
            print(f"    [ESF-MEAS] calibTtag={calibTtag}")

