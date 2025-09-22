import struct

def fusion_mode_str(mode):
    return {
        0: "Initialization",
        1: "Fusion",
        2: "Suspended",
        3: "Disabled"
    }.get(mode, f"Unknown({mode})")

def calib_status_str(val):
    return {
        0: "NOT",
        1: "ING",
        2: "CED",
        3: "TED"
    }.get(val, f"Unknown({val})")

def init_status_str(val):
    return {
        0: "OFF", 
        1: "ING", 
        2: "ZED"
    }.get(val, f"Unknown({val})")

def mnt_alg_status_str(val):
    return {
        0: "OFF",
        1: "ING",
        2: "IED (Coarse?)",
        3: "IED (Fine)"
    }.get(val, f"Unknown({val})")

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

            # InitStatus1 bits
            wtInitStatus    = initStatus1 & 0b11
            mntAlgStatus    = (initStatus1 >> 2) & 0b111
            insInitStatus   = (initStatus1 >> 5) & 0b11

            # InitStatus2 bits
            imuInitStatus = initStatus2 & 0b11

            print(
                f"[ESF-STATUS] iTOW={iTOW} ver={version} fusionMode={fusionMode} ({fusion_mode_str(fusionMode)}) "
                f"wtInit={wtInitStatus}({init_status_str(wtInitStatus)}) "
                f"mntAlg={mntAlgStatus}({mnt_alg_status_str(mntAlgStatus)}) "
                f"insInit={insInitStatus}({init_status_str(insInitStatus)}) "
                f"imuInit={imuInitStatus}({init_status_str(imuInitStatus)}) "
                f"numSens={numSens}"
            )


            # Opakovaná sekce: každý senzor 4B
            if False: #suspended
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
                    fault_flags = []
                    if faults & 0x01: fault_flags.append("badMeas")
                    if faults & 0x02: fault_flags.append("badTTag")
                    if faults & 0x04: fault_flags.append("missingMeas")
                    if faults & 0x08: fault_flags.append("noisyMeas")
                    print(f"    Sensor {i}: type={sensor_type} used={used} ready={ready} calibStatus={calib_status_str(calibStatus)} "
                        f"timeStatus={timeStatus} freq={freq}Hz faults=0x{faults:02X} [{' '.join(fault_flags)}]")

        except Exception as e:
            print(f"[ESF-STATUS] Handler error: {e} | Payload: {payload.hex()}")

