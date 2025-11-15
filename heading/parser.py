# parser.py
"""
Jednoduchý NMEA-like stream parser pro Unicore ASCII věty

Formát:
    # <header> ; <payload> * <CRC32> <CR><LF>

- header : znaky z množiny [0-9 A-Z a-z .,_-]
- payload: znaky z množiny [0-9 A-Z a-z . , - _ " / ; mezera]
- CRC32  : 32-bit CRC (ASCII hex, 8 znaků), přes celý payload (mezi '#' a '*')

- '#'   : začátek věty; pokud dorazí nový '#' dříve než CRLF, zahodíme torzo a
          zvýšíme junk_count.
- Pokud payload obsahuje nepovolený znak -> bad_char_count (+ discard do CRLF nebo nového '#').
- Pokud CRC32 nesedí -> crc32_error_count (+ discard aktuální větu).
- Při úspěchu vrací celé věty včetně řídicích znaků (#…*CRC32\\r\\n).

Ukázka:
#UNIHEADINGA,97,GPS,FINE,2190,365174000,0,0,18,12;INSUFFICIENT_OBS,NONE,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,"",0,0,0,0,0,00,0,0*ee072604\r\n
"""

from __future__ import annotations
from typing import List

from unicore_crc32 import UnicoreCRC32

__all__ = ["UnicoreParser"]


class UnicoreParser:
    # --- Stavový automat ---
    S_FIND_HASH   = 0  # čekáme na '#'
    S_PAYLOAD     = 1  # sbíráme payload (povolené znaky)
    S_CRC32       = 2  # čteme 8 hex číslic CRC32
    S_LF          = 3  # čekáme '\n'
    S_DISCARD     = 4  # zahazujeme do LF nebo nového '#' (po chybě)

    def __init__(self, *, max_payload_len: int = 512):
        self.state = self.S_FIND_HASH

        self._payload = bytearray()     # znaky mezi '#' a '*'
        self._crc32_buf = bytearray()   # 0..8 bajtů ASCII [0-9A-Fa-f]
        self._raw_buf = bytearray()     # celá věta včetně '#', '*', CRC a CRLF

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
        """Zpracuje libovolný chunk bytů. Vrací list validních vět (#…*CRC32\\r\\n)."""
        out: List[bytes] = []
        if not chunk:
            return out

        for b in chunk:
            # -------- S_FIND_HASH --------
            if self.state == self.S_FIND_HASH:
                if b == 0x23:  # '#'
                    self._start_new_raw()
                    self._raw_buf.append(b)
                    self._payload.clear()
                    self._crc32_buf.clear()
                    self.state = self.S_PAYLOAD
                else:
                    # ignoruj šum před začátkem věty
                    continue

            # -------- S_PAYLOAD --------
            elif self.state == self.S_PAYLOAD:
                if b == 0x23:  # nový '#' před CRLF -> předchozí torzo = junk
                    self.junk_count += 1
                    self._start_new_raw()
                    self._raw_buf.append(b)
                    self._payload.clear()
                    self._crc32_buf.clear()
                    # zůstáváme v S_PAYLOAD
                elif b == 0x2A:  # '*': začíná CRC32
                    self._raw_buf.append(b)
                    self._crc32_buf.clear()
                    self.state = self.S_CRC32
                elif self._is_payload_char(b):
                    if len(self._payload) >= self.max_payload_len:
                        # payload přes limit -> zahazuj do LF/nového '#'
                        self.too_long_count += 1
                        self.state = self.S_DISCARD
                    else:
                        self._payload.append(b)
                        self._raw_buf.append(b)
                elif b in (0x0D, 0x0A):  # CR/LF uprostřed payloadu je chyba formátu
                    self.junk_count += 1
                    self.state = self.S_FIND_HASH
                    self._clear_all()
                else:
                    # nepovolený znak v payloadu
                    self.bad_char_count += 1
                    self.state = self.S_DISCARD

            # -------- S_CRC32 (8 hex + CR) --------
            elif self.state == self.S_CRC32:
                if b == 0x23:  # nový '#' uprostřed CRC -> junk + restart
                    self.junk_count += 1
                    self._start_new_raw()
                    self._raw_buf.append(b)
                    self._payload.clear()
                    self._crc32_buf.clear()
                    self.state = self.S_PAYLOAD
                elif self._is_hex(b):
                    if len(self._crc32_buf) < 8:
                        self._crc32_buf.append(b)
                        self._raw_buf.append(b)
                    else:
                        # více než 8 hex znaků -> junk, zahazujeme do LF / '#'
                        self.junk_count += 1
                        self.state = self.S_DISCARD
                elif b == 0x0D:  # CR
                    if len(self._crc32_buf) != 8:
                        # CRC pole není přesně 8 znaků -> junk
                        self.junk_count += 1
                        self.state = self.S_FIND_HASH
                        self._clear_all()
                    else:
                        self._raw_buf.append(b)
                        self.state = self.S_LF
                else:
                    # cokoli jiného v CRC32 je junk
                    self.junk_count += 1
                    self.state = self.S_DISCARD

            # -------- S_LF (čekáme '\n') --------
            elif self.state == self.S_LF:
                if b == 0x0A:  # LF -> konec věty, ověř CRC
                    self._raw_buf.append(b)
                    if self._validate_crc32():
                        out.append(bytes(self._raw_buf))
                    else:
                        self.crc32_error_count += 1
                    # reset pro další větu
                    self.state = self.S_FIND_HASH
                    self._clear_all()
                elif b == 0x23:  # místo LF přišel nový start -> junk + start nové věty
                    self.junk_count += 1
                    self._start_new_raw()
                    self._raw_buf.append(b)
                    self._payload.clear()
                    self._crc32_buf.clear()
                    self.state = self.S_PAYLOAD
                else:
                    # cokoli jiného než LF -> junk
                    self.junk_count += 1
                    self.state = self.S_DISCARD

            # -------- S_DISCARD (zahazujeme po chybě) --------
            elif self.state == self.S_DISCARD:
                # zahazujeme až do LF nebo do nového '#'
                if b == 0x23:  # nový start
                    self._start_new_raw()
                    self._raw_buf.append(b)
                    self._payload.clear()
                    self._crc32_buf.clear()
                    self.state = self.S_PAYLOAD
                elif b == 0x0A:
                    # dorazil LF -> konec rozbité linky, zpět do FIND_HASH
                    self.state = self.S_FIND_HASH
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
        """
        Povolené znaky payloadu:
        - číslice: '0'-'9'
        - písmena: 'A'-'Z', 'a'-'z'
        - interpunkce: '.,-_"/;' + mezera + '/'
        """
        if 0x30 <= b <= 0x39:  # 0-9
            return True
        if 0x41 <= b <= 0x5A:  # A-Z
            return True
        if 0x61 <= b <= 0x7A:  # a-z
            return True
        if b in (0x2E,  # '.'
                 0x2C,  # ','
                 0x2D,  # '-'
                 0x5F,  # '_'
                 0x22,  # '"'
                 0x2F,  # '/'
                 0x3B,  # ';'
                 0x20   # ' '
                 ):
            return True
        return False

    @staticmethod
    def _is_hex(b: int) -> bool:
        """Povolené CRC32 znaky: [0-9A-Fa-f]"""
        if 0x30 <= b <= 0x39:  # 0-9
            return True
        if 0x41 <= b <= 0x46:  # A-F
            return True
        if 0x61 <= b <= 0x66:  # a-f
            return True
        return False

    def _validate_crc32(self) -> bool:
        """
        Ověří CRC32 přes celý rámec (#payload*XXXXXXXX\r\n).
        Pokud je rámec strukturálně špatný, bere se to jako CRC chyba.
        """
        try:
            return self.unicore_crc32.verify_ascii_frame(self._raw_buf)
        except Exception:
            return False


# --- jednoduchý self-test ---
if __name__ == "__main__":
    def make_sentence(payload: bytes) -> bytes:
        """
        Vytvoří validní Unicore ASCII větu:
        b'#' + payload + b'*' + b'%08x' + b'\\r\\n'
        CRC32 se počítá přes payload (bez '#', '*' a CRLF).
        """
        crc = UnicoreCRC32.compute(payload)
        crc_hex = f"{crc:08x}".encode("ascii")
        return b"#" + payload + b"*" + crc_hex + b"\r\n"

    p = UnicoreParser()

    ok1 = make_sentence(b"VERSIONA,79,GPS,FINE,2326,378237000,15434,0,18,889;TEST1,\"A\"")
    ok2 = make_sentence(b"UNIHEADINGA,97,GPS,FINE,2190,365174000,0,0,18,12;INSUFFICIENT_OBS,NONE,0.0")
    bad_char = b"##ABC,12\x01*00000000\r\n"          # 0x01 -> bad_char
    bad_crc  = b"#ABC,123*00000000\r\n"            # špatné CRC (formátově OK)
    junk_mid = b"noise#ABC,1\r\n#VEL,2*00000000\r\n"  # první věta rozbitá, druhá OK formátově, CRC špatné

    # valid po junku:
    ok3 = make_sentence(b"MSM,7F;OK")
    ok4 = b'#UNIHEADINGA,97,GPS,FINE,2190,365174000,0,0,18,12;INSUFFICIENT_OBS,NONE,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,"",0,0,0,0,0,00,0,0*ee072604\r\n'
    frame2 = b'#OBSVHA,97,GPS,FINE,2190,359897000,0,0,18,14;0*9d38304c\r\n'
    frame3 = b'#UNIHEADINGA,92,GPS,FINE,2392,517944600,0,0,18,6;INSUFFICIENT_OBS,NONE,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,"",0,0,0,0,0,00,0,0*044457ef\r\n'
    stream = ok1 + bad_char + bad_crc + junk_mid + ok3 + ok2 + ok4 + frame2 + frame3

    out = p.feed(stream)
    for i, s in enumerate(out, 1):
        print(i, s)

    print("junk      :", p.junk_count)
    print("bad_char  :", p.bad_char_count)
    print("crc32_err :", p.crc32_error_count)
    print("too_long  :", p.too_long_count)
    print("parsed    :", p.senetces_parsed)
