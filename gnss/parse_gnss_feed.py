#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys

# ----------------------- Stav parseru -----------------------

class GnssParseResult:
    PROCESSING = 'processing'         # Ještě nesložená věta
    NMEA = 'nmea'                     # Validní NMEA věta
    UBX = 'ubx'                       # Validní UBX zpráva
    CORRUPTED = 'corrupted'           # Rozpoznaná nekorektní sekvence / desync / junk blok
    CHECKSUM_ERROR = 'checksum_error' # Špatný checksum

# ----------------------- Hlavní parser -----------------------

class GnssStreamParser:
    """
    Kombinovaný byte-by-byte parser pro GNSS stream obsahující NMEA i UBX věty.
    Důraz na time-critical feed(): vše počítá průběžně a pouze z přicházejících bajtů.
    """
    UBX_SYNC_1 = 0xB5
    UBX_SYNC_2 = 0x62

    def __init__(self, max_ubx_payload: int = 8192, junk_flush_len: int = 64):
        # Obecný stav
        self.state = 'IDLE'
        # NMEA
        self.nmea_buf = bytearray()
        self.nmea_running_xor = 0
        self.nmea_hex1 = None
        self.nmea_hex2 = None
        # UBX
        self.ubx_header = bytearray()
        self.ubx_payload = bytearray()
        self.ubx_cksum = bytearray()
        self.expected_payload_len = 0
        self.max_ubx_payload = max_ubx_payload
        # JUNK
        self.junk_buf = bytearray()
        self.junk_flush_len = junk_flush_len  # flush po dosažení této délky

    # ---------- Pomůcky ----------
    @staticmethod
    def _hex_nibble(b: int):
        """Vrátí hodnotu hex nibbla pro ASCII 0-9A-Fa-f, jinak None."""
        if 48 <= b <= 57:   # '0'-'9'
            return b - 48
        if 65 <= b <= 70:   # 'A'-'F'
            return b - 55
        if 97 <= b <= 102:  # 'a'-'f'
            return b - 87
        return None

    @staticmethod
    def _ubx_checksum(data: bytes):
        ck_a = 0
        ck_b = 0
        for b in data:
            ck_a = (ck_a + b) & 0xFF
            ck_b = (ck_b + ck_a) & 0xFF
        return (ck_a, ck_b)

    # ---------- Interní startery ----------
    def _start_nmea(self):
        self.nmea_buf = bytearray([ord('$')])
        self.nmea_running_xor = 0
        self.nmea_hex1 = None
        self.nmea_hex2 = None
        self.state = 'NMEA_HEADER'

    def _start_ubx(self):
        self.state = 'UBX_SYNC2'

    # ---------- API ----------
    def feed(self, b: int):
        """
        Posune stav parseru o jeden byte.
        Vrací tuple: (typ, data_bytes | None)
        typ ∈ {processing, nmea, ubx, corrupted, checksum_error}
        data_bytes jsou kompletní byty dané zprávy (kopie až při dokončení věty).
        """
        # --------- Čekání na začátky věty / start JUNK ----------
        if self.state == 'IDLE':
            if b == ord('$'):
                self._start_nmea()
                return (GnssParseResult.PROCESSING, None)
            elif b == self.UBX_SYNC_1:
                self._start_ubx()
                return (GnssParseResult.PROCESSING, None)
            else:
                # start agregovaného JUNKu
                self.junk_buf = bytearray([b])
                self.state = 'JUNK'
                return (GnssParseResult.PROCESSING, None)

        # --------- Agregovaný JUNK blok ----------
        elif self.state == 'JUNK':
            # pokud narazíme na start věty, flush JUNK a současně interně nastartujeme novou větu
            if b == ord('$') or b == self.UBX_SYNC_1:
                out = bytes(self.junk_buf)
                # nastartuj novou větu uvnitř tohoto feedu (start-bajt neztrácíme)
                if b == ord('$'):
                    self._start_nmea()
                else:
                    self._start_ubx()
                # vyprázdni junk
                self.junk_buf = bytearray()
                # signalizuj callerovi junk blok; další feed() bude plynule pokračovat v nové větě
                return (GnssParseResult.CORRUPTED, out)
            else:
                self.junk_buf.append(b)
                if len(self.junk_buf) >= self.junk_flush_len:
                    out = bytes(self.junk_buf)
                    self.junk_buf = bytearray()
                    self.state = 'IDLE'
                    return (GnssParseResult.CORRUPTED, out)
                return (GnssParseResult.PROCESSING, None)

        # --------- Parsování NMEA věty (byte-based XOR, bez decode) ----------
        elif self.state == 'NMEA_HEADER':
            # Očekáváme dalších 5 znaků hlavičky ($ + 5 = 6 bajtů celkem)
            self.nmea_buf.append(b)
            # zahrnout hlavičku do XOR (vše mezi '$' a '*')
            if len(self.nmea_buf) >= 2 and len(self.nmea_buf) <= 6:
                self.nmea_running_xor ^= b

            if len(self.nmea_buf) == 6:
                # Validace hlavičky: $TTSSS (TT=alnum, SSS=alnum)
                t1, t2, s1, s2, s3 = self.nmea_buf[1:6]
                if not (chr(t1).isalnum() and chr(t2).isalnum() and
                        chr(s1).isalnum() and chr(s2).isalnum() and chr(s3).isalnum()):
                    buf = bytes(self.nmea_buf)
                    self.state = 'IDLE'
                    return (GnssParseResult.CORRUPTED, buf)
                self.state = 'NMEA_BODY'
            return (GnssParseResult.PROCESSING, None)

        elif self.state == 'NMEA_BODY':
            self.nmea_buf.append(b)
            if b == ord('*'):
                # Hvězdička ukončuje část pro XOR – nic dalšího už do XOR nepřidáváme
                self.state = 'NMEA_CSUM1'
            else:
                # Průběžný XOR (jen mezi '$' a '*', hvězdička se už nepřidává)
                self.nmea_running_xor ^= b
            return (GnssParseResult.PROCESSING, None)

        elif self.state == 'NMEA_CSUM1':
            self.nmea_buf.append(b)
            nib = self._hex_nibble(b)
            if nib is None:
                buf = bytes(self.nmea_buf)
                self.state = 'IDLE'
                return (GnssParseResult.CORRUPTED, buf)
            self.nmea_hex1 = nib
            self.state = 'NMEA_CSUM2'
            return (GnssParseResult.PROCESSING, None)

        elif self.state == 'NMEA_CSUM2':
            self.nmea_buf.append(b)
            nib = self._hex_nibble(b)
            if nib is None:
                buf = bytes(self.nmea_buf)
                self.state = 'IDLE'
                return (GnssParseResult.CORRUPTED, buf)
            self.nmea_hex2 = nib
            self.state = 'NMEA_END_CR'
            return (GnssParseResult.PROCESSING, None)

        elif self.state == 'NMEA_END_CR':
            self.nmea_buf.append(b)
            if b != 0x0D:
                buf = bytes(self.nmea_buf)
                self.state = 'IDLE'
                return (GnssParseResult.CORRUPTED, buf)
            self.state = 'NMEA_END_LF'
            return (GnssParseResult.PROCESSING, None)

        elif self.state == 'NMEA_END_LF':
            self.nmea_buf.append(b)
            buf = bytes(self.nmea_buf)
            self.state = 'IDLE'
            if b != 0x0A:
                return (GnssParseResult.CORRUPTED, buf)
            # Zkompletováno: ověř checksum
            rx = (self.nmea_hex1 << 4) | self.nmea_hex2
            if rx == self.nmea_running_xor:
                return (GnssParseResult.NMEA, buf)
            else:
                return (GnssParseResult.CHECKSUM_ERROR, buf)

        # --------- Parsování UBX zprávy ----------
        elif self.state == 'UBX_SYNC2':
            if b == self.UBX_SYNC_2:
                self.ubx_header = bytearray()
                self.state = 'UBX_HEADER'
            else:
                self.state = 'IDLE'
                return (GnssParseResult.CORRUPTED, bytes([self.UBX_SYNC_1, b]))
            return (GnssParseResult.PROCESSING, None)

        elif self.state == 'UBX_HEADER':
            self.ubx_header.append(b)
            if len(self.ubx_header) == 4:
                # header: class, id, len_lo, len_hi
                self.expected_payload_len = self.ubx_header[2] | (self.ubx_header[3] << 8)
                if self.expected_payload_len > self.max_ubx_payload:
                    bad = bytes([self.UBX_SYNC_1, self.UBX_SYNC_2] + list(self.ubx_header))
                    self.state = 'IDLE'
                    return (GnssParseResult.CORRUPTED, bad)
                self.ubx_payload = bytearray()
                self.state = 'UBX_PAYLOAD'
            return (GnssParseResult.PROCESSING, None)

        elif self.state == 'UBX_PAYLOAD':
            self.ubx_payload.append(b)
            if len(self.ubx_payload) == self.expected_payload_len:
                self.ubx_cksum = bytearray()
                self.state = 'UBX_CKSUM1'
            return (GnssParseResult.PROCESSING, None)

        elif self.state == 'UBX_CKSUM1':
            self.ubx_cksum.append(b)
            if len(self.ubx_cksum) == 1:
                self.state = 'UBX_CKSUM2'
            return (GnssParseResult.PROCESSING, None)

        elif self.state == 'UBX_CKSUM2':
            self.ubx_cksum.append(b)
            full = bytes([self.UBX_SYNC_1, self.UBX_SYNC_2] +
                         list(self.ubx_header) + list(self.ubx_payload) +
                         list(self.ubx_cksum))
            self.state = 'IDLE'
            if len(self.ubx_cksum) == 2:
                ck_a, ck_b = self._ubx_checksum(self.ubx_header + self.ubx_payload)
                if ck_a == self.ubx_cksum[0] and ck_b == self.ubx_cksum[1]:
                    return (GnssParseResult.UBX, full)
                else:
                    return (GnssParseResult.CHECKSUM_ERROR, full)
            return (GnssParseResult.CORRUPTED, full)

        # --------- Fallback ----------
        return (GnssParseResult.PROCESSING, None)


# ---------------------- Unit test ----------------------

def _nmea_build(body_ascii: str) -> bytes:
    """
    Pomocná funkce pro stavbu NMEA věty: vstup je text mezi '$' a '*' (bez nich),
    funkce vrátí kompletní byty se správným checksumem a CRLF.
    """
    xorv = 0
    for ch in body_ascii.encode('ascii'):
        xorv ^= ch
    return b'$' + body_ascii.encode('ascii') + b'*' + f"{xorv:02X}".encode('ascii') + b"\r\n"

def main():
    """
    Jednoduchý test runner – simulace NMEA, UBX i chyb.
    """
    p = GnssStreamParser()

    print("--- TEST NMEA OK ---")
    nmea_ok = _nmea_build("GPGGA,1234,5678")  # checksum = 0x5E
    for b in nmea_ok:
        typ, msg = p.feed(b)
        if typ != GnssParseResult.PROCESSING:
            print(f"TYPE: {typ}, MSG: {msg}")

    print("--- TEST NMEA CHKSUM ERROR ---")
    nmea_bad = bytearray(nmea_ok)
    if len(nmea_bad) >= 4:
        # poškodíme poslední hex digit checksumu
        if nmea_bad[-4] in b"0123456789ABCDEF":
            nmea_bad[-4] = ord('0') if nmea_bad[-4] == ord('F') else (nmea_bad[-4] + 1)
    for b in bytes(nmea_bad):
        typ, msg = p.feed(b)
        if typ != GnssParseResult.PROCESSING:
            print(f"TYPE: {typ}, MSG: {msg}")

    print("--- TEST UBX OK ---")
    payload = [0xAA, 0xBB, 0xCC, 0xDD]
    header = [0x01, 0x02, 0x04, 0x00]
    data = bytearray(header + payload)
    ck_a, ck_b = GnssStreamParser._ubx_checksum(data)
    ubx = bytes([0xB5, 0x62] + header + payload + [ck_a, ck_b])
    for b in ubx:
        typ, msg = p.feed(b)
        if typ != GnssParseResult.PROCESSING:
            print(f"TYPE: {typ}, MSG: {msg}")

    print("--- TEST UBX CHKSUM ERROR ---")
    ubx_bad = bytes([0xB5, 0x62] + header + payload + [(ck_a + 1) & 0xFF, ck_b])
    for b in ubx_bad:
        typ, msg = p.feed(b)
        if typ != GnssParseResult.PROCESSING:
            print(f"TYPE: {typ}, MSG: {msg}")

    print("--- TEST RANDOM JUNK ---")
    junk = b"XQWERTY12345" + b"$"
    for b in junk:
        typ, msg = p.feed(b)
        if typ != GnssParseResult.PROCESSING:
            print(f"TYPE: {typ}, MSG: {msg}")

    print("--- TEST NMEA IN RANDOM JUNK ---")
    mix = b"junk1" + _nmea_build("GPRMC,1,2,3") + b"junk2" + b"$"
    for b in mix:
        typ, msg = p.feed(b)
        if typ != GnssParseResult.PROCESSING:
            print(f"TYPE: {typ}, MSG: {msg}")

    print("--- TEST UBX IN RANDOM JUNK ---")
    # Použijeme ten samý ubx z výše
    mix2 = b"abc" + ubx + b"def" + b"$"
    for b in mix2:
        typ, msg = p.feed(b)
        if typ != GnssParseResult.PROCESSING:
            print(f"TYPE: {typ}, MSG: {msg}")


if __name__ == "__main__":
    main()
