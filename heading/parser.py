# parser.py
"""
Jednoduchý NMEA-like stream parser

Formát:
    # <header> ; <payload> * <CRC32> <CR><LF>

- header : znaky z množiny [0-9 A-Z a-z , . _ - ]
- payload: znaky z množiny [0-9 A-Z , . _ - "]
- CRC32  : 32-bit CRC (ASCII and binary only), přes header a payload (bez '#' a '*') 

- '#'   : začátek věty; pokud dorazí nový '#' dříve než CRLF, zahodíme torzo a
          zvýšíme junk_count.
- Pokud payload obsahuje nepovolený znak -> bad_char_count (+ discard do CRLF nebo nového '#').
- Pokud CS nesedí -> cs_error_count (+ discard aktuální větu).
- Při úspěchu vrací celé věty včetně řídicích znaků (#…*CRC32\r\n).

Ukázka:
#UNIHEADINGA,97,GPS,FINE,2190,365174000,0,0,18,12;INSUFFICIENT_OBS,NONE,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,"",0,0,0,0,0,00,0,0*ee072604

"""

from __future__ import annotations
from typing import List

from unicore_crc32 import UnicoreCRC32

__all__ = ["UnicoreParser"]


class UnicoreParser:
    # --- Stavový automat ---
    S_FIND_HASH   = 0  # čekáme na '#'
    S_PAYLOAD     = 1  # sbíráme payload (povolené znaky)
    S_CRC32       = 2  # čteme 4 hex číslice CRC32
    S_CR          = 3  # čekáme '\r'
    S_LF          = 4  # čekáme '\n'
    S_DISCARD     = 5  # zahazujeme do CRLF nebo nového '#' (po chybě)

    def __init__(self, *, max_payload_len: int = 512):
        self.state = self.S_FIND_HASH

        self._payload = bytearray()
        self._crc32_buf  = bytearray()   # 0..4 bajty ASCII [0-9a-f]
        self._raw_buf = bytearray()   # pro rekonstrukci celé věty

        self.max_payload_len = max_payload_len

        self.unicore_crc32 = UnicoreCRC32()

        # Counters
        self.junk_count = 0
        self.bad_char_count = 0
        self.crc32_error_count = 0
        self.too_long_count = 0
        self.senetces_parsed = 0

    # --- Public API ---
    def reset(self) -> None:
        self.state = self.S_FIND_HASH
        self._payload.clear()
        self._crc32_buf.clear()
        self._raw_buf.clear()
        self.junk_count = 0
        self.bad_char_count = 0
        self.crc32_error_count = 0
        self.too_long_count = 0
        self.senetces_parsed = 0

    def feed(self, chunk: bytes) -> List[bytes]:
        """Zpracuje libovolný chunk bytů. Vrací list validních vět ($…*CS\\r\\n)."""
        out: List[bytes] = []
        if not chunk:
            return out

        for b in chunk:
            if self.state == self.S_FIND_HASH:
                if b == 0x24:  # '#'
                    self._start_new_raw()
                    self._raw_buf.append(b)
                    self.state = self.S_PAYLOAD
                else:
                    # ignoruj šum před začátkem věty
                    continue

            elif self.state == self.S_PAYLOAD:
                if b == 0x24:  # nový '$' před CRLF -> junk
                    self.junk_count += 1
                    self._start_new_raw()
                    self._raw_buf.append(b)
                    self._payload.clear()
                    self._cs_buf.clear()
                    # zůstáváme v S_PAYLOAD
                elif b == 0x2A:  # '*'
                    self._raw_buf.append(b)
                    self._cs_buf.clear()
                    self.state = self.S_CS
                elif self._is_payload_char(b):
                    if len(self._payload) >= self.max_payload_len:
                        # payload přes limit -> zahazuj do CRLF/nového '$'
                        self.too_long_count += 1
                        self.state = self.S_DISCARD
                    else:
                        self._payload.append(b)
                        self._raw_buf.append(b)
                elif b in (0x0D, 0x0A):  # CR/LF v payloadu je chyba formátu
                    self.junk_count += 1
                    self.state = self.S_FIND_DOLLAR
                    self._payload.clear()
                    self._cs_buf.clear()
                    self._raw_buf.clear()
                else:
                    # nepovolený znak v payloadu
                    self.bad_char_count += 1
                    self.state = self.S_DISCARD

            elif self.state == self.S_CS:
                if b == 0x24:  # nový '$' uprostřed CS -> junk + restart
                    self.junk_count += 1
                    self._start_new_raw()
                    self._raw_buf.append(b)
                    self._payload.clear()
                    self._cs_buf.clear()
                    self.state = self.S_PAYLOAD
                elif self._is_hex(b):
                    if len(self._cs_buf) < 2:
                        self._cs_buf.append(b)
                        self._raw_buf.append(b)
                    else:
                        # více než 2 hexy do CS -> junk
                        self.junk_count += 1
                        self.state = self.S_DISCARD
                elif b == 0x0D:  # CR
                    if len(self._cs_buf) != 2:
                        self.junk_count += 1
                        self.state = self.S_FIND_DOLLAR
                        self._clear_all()
                    else:
                        self._raw_buf.append(b)
                        self.state = self.S_LF
                else:
                    # cokoliv jiného v CS je junk (formát CS řešíme striktně)
                    self.junk_count += 1
                    self.state = self.S_DISCARD

            elif self.state == self.S_LF:
                if b == 0x0A:  # LF -> konec věty, ověř CS
                    self._raw_buf.append(b)
                    if self._validate_checksum():
                        out.append(bytes(self._raw_buf))
                    else:
                        self.cs_error_count += 1
                    # reset pro další větu
                    self.state = self.S_FIND_DOLLAR
                    self._clear_all()
                elif b == 0x24:  # místo LF přišel nový start -> junk + start nové věty
                    self.junk_count += 1
                    self._start_new_raw()
                    self._raw_buf.append(b)
                    self._payload.clear()
                    self._cs_buf.clear()
                    self.state = self.S_PAYLOAD
                else:
                    # cokoli jiného než LF -> junk
                    self.junk_count += 1
                    self.state = self.S_DISCARD

            elif self.state == self.S_DISCARD:
                # zahazujeme až do CRLF nebo do nového '$'
                if b == 0x24:  # nový start
                    self._start_new_raw()
                    self._raw_buf.append(b)
                    self._payload.clear()
                    self._cs_buf.clear()
                    self.state = self.S_PAYLOAD
                elif b == 0x0A:
                    # dorazil LF -> konec rozbité linky, zpět do FIND_DOLLAR
                    self.state = self.S_FIND_DOLLAR
                    self._clear_all()
                else:
                    # jinak jen zahazujeme
                    pass

        self.senetces_parsed += len(out)
        return out

    # --- helpers ---
    def _start_new_raw(self) -> None:
        self._raw_buf.clear()

    def _clear_all(self) -> None:
        self._payload.clear()
        self._crc32_buf.clear()
        self._raw_buf.clear()

    @staticmethod
    def _is_payload_char(b: int) -> bool:
        """Povolené znaky payloadu: [0-9 A-Z a-z , - . _ "]"""
        return (48 <= b <= 57) or (65 <= b <= 90) or b in (44, 45)

    @staticmethod
    def _is_hex(b: int) -> bool:
        """Povolené CS znaky: [0-9 a-f]"""
        return (48 <= b <= 57) or (65 <= b <= 70)

    def _validate_checksum(self) -> bool:
        """CRC32 přes payload"""
        return self.unicore_crc32.verify_ascii_frame(self._payload)

# --- jednoduchý self-test ---
if __name__ == "__main__":
    def make_sentence(payload: bytes) -> bytes:
        cs = 0
        for b in payload:
            cs ^= b
        cs_hex = f"{cs:02X}".encode("ascii")
        return b"$" + payload + b"*" + cs_hex + b"\r\n"

    p = UnicoreParser()

    ok1 = make_sentence(b"ABC,123")
    ok2 = make_sentence(b"VEL-1,XYZ")
    bad_char = b"$ABC,12z*00\r\n"          # 'z' -> bad_char
    bad_cs   = b"$ABC,123*00\r\n"          # špatné CS
    junk_mid = b"salkdhaslj$\r\nABC,1" + b"$" + b"VEL,2*00\r\n"  # nový '$' uprostřed
    # valid po junku:
    ok3 = make_sentence(b"MSM,7F")

    stream = ok1 + bad_char + bad_cs + junk_mid + ok3 + ok2

    out = p.feed(stream)
    for i, s in enumerate(out, 1):
        print(i, s)

    print("junk      :", p.junk_count)
    print("bad_char  :", p.bad_char_count)
    print("cs_error  :", p.cs_error_count)
    print("too_long  :", p.too_long_count)
    print("parsed    :", p.senetces_parsed)
