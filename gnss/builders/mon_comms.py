from .ubx_utils import ubx_packet

def build_mon_comms_poll() -> bytes:
    # UBX-MON-COMMS poll (class 0x0A, id 0x36)
    return ubx_packet(0x0A, 0x36, b'')
