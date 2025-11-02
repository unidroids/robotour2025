# build_odm.py
import struct
from .ubx_utils import ubx_packet

# UBX-ESF-MEAS dataType pro rychlost
DATA_TYPE_SPEED = 11  # m/s * 1e-3 (tj. mm/s), signed, 24 bity

def _pack_signed24(value: int) -> int:
    """
    Zabalí celé číslo do 24bitového signed two's complement pole (bits 23..0).
    Vrací hodnotu maskovanou na 24 bitů.
    """
    if not isinstance(value, int):
        value = int(round(value))
    # Rozsah signed 24bit: -2^23 .. 2^23 - 1  ->  -8388608 .. 8388607
    if value < -(1 << 23) or value > (1 << 23) - 1:
        raise ValueError("speed (mm/s) mimo rozsah 24bit signed (-8388608..8388607)")
    return value & 0xFFFFFF

def build_odm(mono: int, speed: int) -> bytes:
    """
    Vytvoří UBX-ESF-MEAS (0x10 0x02) zprávu s jedním měřením typu 'speed' (dataType=11).

    Parametry:
        mono  : U4 timeTag [ms] – časový tag měření (externí senzor)
        speed : signed 24bit v mm/s – rychlost (m/s * 1e-3)

    Návrat:
        bytes – kompletní UBX paket (včetně hlavičky a checksumu)

    Poznámky:
    - flags.bits15..11 = numMeas = 1
    - ostatní bity flags jsou 0 (timeMarkSent/timeMarkEdge nepoužity)
    - id (provider) = 0 (můžeš si případně upravit)
    - data word: bits 0..23 = signed 24bit rychlosti (two's complement)
                  bits 24..29 = dataType (11)
                  bits 30..31 = 0
    """
    # --- hlavička payloadu ---
    time_tag = int(mono) & 0xFFFFFFFF

    num_meas = 1
    flags = (num_meas << 11)  # bits 15..11 numMeas
    provider_id = 0           # můžeš dle potřeby změnit

    # --- měření: speed ---
    data_field = _pack_signed24(int(speed))  # mm/s, signed 24bit
    data_word = data_field | ((DATA_TYPE_SPEED & 0x3F) << 24)

    # --- sestavení payloadu ---
    # <I H H  I  => timeTag(U4), flags(U2), id(U2), data(X4)
    payload = struct.pack('<I H H I', time_tag, flags, provider_id, data_word)

    # UBX-ESF-MEAS
    msg_class = 0x10
    msg_id = 0x02
    return ubx_packet(msg_class, msg_id, payload)


__all__ = ["build_odm"]
