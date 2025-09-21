from .ubx_utils import ubx_packet
import struct

# klíč pro CFG-RATE-NAV_PRIO z dokumentace
CFG_RATE_NAV_PRIO = 0x20210004  # 4 byty LE

def build_prio_rate_valset(on: bool) -> bytes:
    """
    Vytvoří UBX-CFG-VALSET pro změnu CFG-RATE-NAV_PRIO.
    on=True:  nastaví na 30 Hz (doporučená max rychlost pro F9R)
    on=False: nastaví na 0 Hz (vypne PRIO zprávy)
    """
    version = 0x00
    layers = 0x01  # RAM
    reserved0 = b'\x00\x00'
    payload = struct.pack('<BB2s', version, layers, reserved0)
    # Přidej KeyID (4B LE) a value (1B), musí být v pořadí key-value!
    value = 30 if on else 0
    key = struct.pack('<I', CFG_RATE_NAV_PRIO)
    payload += key + struct.pack('<B', value)
    return ubx_packet(0x06, 0x8a, payload)

def build_prio_on():
    return build_prio_rate_valset(True)

def build_prio_off():
    return build_prio_rate_valset(False)

if __name__ == '__main__':
    print("PRIO ON:", build_prio_on().hex())
    print("PRIO OFF:", build_prio_off().hex())
