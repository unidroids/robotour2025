from .ubx_utils import ubx_packet

def build_esf_status_poll() -> bytes:
    # UBX-ESF-STATUS poll (class 0x10, id 0x10)
    return ubx_packet(0x10, 0x10, b'')

if __name__ == '__main__':
    print("ESF-STATUS poll:", build_esf_status_poll().hex())
