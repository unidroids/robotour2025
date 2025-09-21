import struct
from .ubx_utils import ubx_packet

def build_odo(cmd_line: str) -> bytes:
    """
    Převede příkazový řádek ODO na UBX-ESF-MEAS packet.
    cmd_line: např. 'ODO 19BA73A4 000011E1 0 000013DF 0'
    Vrací: UBX packet (bytes)
    """

    parts = cmd_line.strip().split()
    if len(parts) != 6 or parts[0] != "ODO":
        raise ValueError("ODO: invalid format. Expect 'ODO <time_tag> <left_ticks> <left_dir> <right_ticks> <right_dir>'")

    try:
        # Parse vstupů (vždy hex kromě dir)
        time_tag = int(parts[1], 16)
        left_ticks = int(parts[2], 16)
        left_dir = int(parts[3], 10)
        right_ticks = int(parts[4], 16)
        right_dir = int(parts[5], 10)
    except Exception as e:
        raise ValueError(f"ODO: parse error: {e}")

    # --- UBX-ESF-MEAS payload (dva měřené kanály: left/right) ---
    # viz UBX dokumentace
    def pack_tick(ticks, direction, data_type):
        data = (ticks & 0x7FFFFF)  # 23 bitů
        data |= (int(direction) & 0x1) << 23
        data |= (data_type & 0x3F) << 24
        return data

    meas_left = pack_tick(left_ticks, left_dir, 8)   # typ 8 = rear-left
    meas_right = pack_tick(right_ticks, right_dir, 9) # typ 9 = rear-right

    num_meas = 2
    flags = (num_meas << 11)   # bits 15…11 numMeas
    provider_id = 0            # prozatím staticky, lze doplnit

    payload = struct.pack('<I H H', time_tag, flags, provider_id) \
        + struct.pack('<I', meas_left) \
        + struct.pack('<I', meas_right)

    msg_class = 0x10
    msg_id = 0x02

    return ubx_packet(msg_class, msg_id, payload)
