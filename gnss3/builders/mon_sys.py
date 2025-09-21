from .ubx_utils import ubx_packet

def build_mon_sys_poll() -> bytes:
    # UBX-MON-SYS poll (bez payloadu)
    msg_class = 0x0A
    msg_id = 0x39
    payload = b''
    return ubx_packet(msg_class, msg_id, payload)

# Test
if __name__ == '__main__':
    print("MON-SYS poll:", build_mon_sys_poll().hex())
