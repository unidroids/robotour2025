import struct

# Pro čitelnější typy (viz tabulka 15 v manuálu)
ESF_RAW_TYPE = {
    5:  "gyroZ",
    12: "tempGyro",
    13: "gyroY",
    14: "gyroX",
    16: "accX",
    17: "accY",
    18: "accZ",
    # ... další můžeš přidat dle potřeby
}

def decode_data(dtype, value):
    # Default: vrátí hex
    if dtype == 16 or dtype == 17 or dtype == 18:  # acc
        return f"{value} ({value/1024:.4f} m/s²)"
    if dtype == 5 or dtype == 13 or dtype == 14:  # gyro
        # deg/s × 2^-12 (viz manuál)
        return f"{value} ({value/4096:.4f} deg/s)"
    if dtype == 12:  # tempGyro
        return f"{value} ({value/100:.2f}°C)"
    return f"{value} (raw)"
    

class EsfRawHandler:
    def __init__(self):
        self.count = 0

    def handle(self, msg_class, msg_id, payload):
        self.count += 1
        if len(payload) < 4:
            print(f"[ESF-RAW] Too short: {len(payload)} bytes")
            return
        # první 4B jsou reserved
        reserved = payload[:4]
        print(f"[ESF-RAW] #{self.count} reserved0={reserved.hex()} total_payload={len(payload)}B")

        N = (len(payload) - 4) // 8
        for i in range(N):
            base = 4 + i*8
            if len(payload) < base+8:
                print(f"  [ESF-RAW] Incomplete data at {i}: {payload[base:].hex()}")
                continue
            # data: 4B (little endian: nejnižší 3B = dataField, nejvyšší = type)
            data_raw = payload[base:base+4]
            data = int.from_bytes(data_raw, 'little', signed=False)
            dataField = data & 0xFFFFFF
            # Pokud je typ signed, bude potřeba převést na 24bit signed int:
            # 24bit sign extension:
            if dataField & 0x800000:
                dataField -= 1<<24
            dataType = (data >> 24) & 0xFF
            sTtag, = struct.unpack('<I', payload[base+4:base+8])
            type_str = ESF_RAW_TYPE.get(dataType, f"type{dataType}")
            value_decoded = decode_data(dataType, dataField)
            print(
                f"  [ESF-RAW] {i}: type={type_str} ({dataType}) "
                f"data=0x{dataField & 0xFFFFFF:06X} ({value_decoded}) "
                f"sTtag={sTtag}"
            )
