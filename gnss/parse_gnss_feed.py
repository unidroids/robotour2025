#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Kombinovaný byte-by-byte parser pro GNSS stream s NMEA a UBX zprávami.
# Time-critical: feed() je O(1), průběžný XOR pro NMEA, bez zbytečných průchodů.

class GnssParseResult:
    PROCESSING = 'processing'
    NMEA = 'nmea'
    UBX = 'ubx'
    CORRUPTED = 'corrupted'
    CHECKSUM_ERROR = 'checksum_error'


class GnssStreamParser:
    UBX_SYNC_1 = 0xB5
    UBX_SYNC_2 = 0x62

    def __init__(self, max_ubx_payload: int = 512, junk_flush_len: int = 64):
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
        self.junk_flush_len = junk_flush_len

    @staticmethod
    def _hex_nibble(b: int):
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

    # starters
    def _start_nmea(self):
        self.nmea_buf = bytearray([ord('$')])
        self.nmea_running_xor = 0
        self.nmea_hex1 = None
        self.nmea_hex2 = None
        self.state = 'NMEA_HEADER'

    def _start_ubx(self):
        self.state = 'UBX_SYNC2'

    def feed(self, b: int):
        # IDLE
        if self.state == 'IDLE':
            if b == ord('$'):
                self._start_nmea()
                return (GnssParseResult.PROCESSING, None)
            elif b == self.UBX_SYNC_1:
                self._start_ubx()
                return (GnssParseResult.PROCESSING, None)
            else:
                self.junk_buf = bytearray([b])
                self.state = 'JUNK'
                return (GnssParseResult.PROCESSING, None)

        # JUNK
        elif self.state == 'JUNK':
            if b == ord('$') or b == self.UBX_SYNC_1:
                out = bytes(self.junk_buf)
                if b == ord('$'):
                    self._start_nmea()
                else:
                    self._start_ubx()
                self.junk_buf = bytearray()
                return (GnssParseResult.CORRUPTED, out)
            else:
                self.junk_buf.append(b)
                if len(self.junk_buf) >= self.junk_flush_len:
                    out = bytes(self.junk_buf)
                    self.junk_buf = bytearray()
                    self.state = 'IDLE'
                    return (GnssParseResult.CORRUPTED, out)
                return (GnssParseResult.PROCESSING, None)

        # NMEA
        elif self.state == 'NMEA_HEADER':
            # ochrana: nový start uprostřed hlavičky
            if b == ord('$'):
                buf = bytes(self.nmea_buf) if self.nmea_buf else b'$'
                self._start_nmea()
                return (GnssParseResult.CORRUPTED, buf)

            if b == self.UBX_SYNC_1:
                buf = bytes(self.nmea_buf) if self.nmea_buf else b'$'
                self._start_ubx()
                return (GnssParseResult.CORRUPTED, buf)

            self.nmea_buf.append(b)
            # XOR hlavičky (mezi '$' a '*')
            if 2 <= len(self.nmea_buf) <= 6:
                self.nmea_running_xor ^= b

            if len(self.nmea_buf) == 6:
                t1, t2, s1, s2, s3 = self.nmea_buf[1:6]
                def isAZ(x): return 65 <= x <= 90
                if not (isAZ(t1) and isAZ(t2) and isAZ(s1) and isAZ(s2) and isAZ(s3)):
                    buf = bytes(self.nmea_buf)
                    self.state = 'IDLE'
                    return (GnssParseResult.CORRUPTED, buf)
                self.state = 'NMEA_BODY'
            return (GnssParseResult.PROCESSING, None)

        elif self.state == 'NMEA_BODY':
            # nový start uprostřed těla
            if b == ord('$'):
                buf = bytes(self.nmea_buf)
                self._start_nmea()
                return (GnssParseResult.CORRUPTED, buf)
            if b == self.UBX_SYNC_1:
                buf = bytes(self.nmea_buf)
                self._start_ubx()
                return (GnssParseResult.CORRUPTED, buf)

            self.nmea_buf.append(b)
            if b == ord('*'):
                self.state = 'NMEA_CSUM1'
            else:
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
            rx = (self.nmea_hex1 << 4) | self.nmea_hex2
            if rx == self.nmea_running_xor:
                return (GnssParseResult.NMEA, buf)
            else:
                return (GnssParseResult.CHECKSUM_ERROR, buf)

        # UBX
        elif self.state == 'UBX_SYNC2':
            if b == self.UBX_SYNC_2:
                self.ubx_header = bytearray()
                self.state = 'UBX_HEADER'
                return (GnssParseResult.PROCESSING, None)
            else:
                # pokud druhý sync selže a dorazilo '$', nezaříznout ho – přepnout na NMEA
                if b == ord('$'):
                    self._start_nmea()
                    self.state = 'NMEA_HEADER'
                    return (GnssParseResult.CORRUPTED, bytes([self.UBX_SYNC_1]))
                # jinak desync
                self.state = 'IDLE'
                return (GnssParseResult.CORRUPTED, bytes([self.UBX_SYNC_1, b]))

        elif self.state == 'UBX_HEADER':
            self.ubx_header.append(b)
            if len(self.ubx_header) == 4:
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

        return (GnssParseResult.PROCESSING, None)


# ---------- Unit test (main) ----------

def _nmea_build(body_ascii: str) -> bytes:
    xorv = 0
    for ch in body_ascii.encode('ascii'):
        xorv ^= ch
    return b'$' + body_ascii.encode('ascii') + b'*' + f"{xorv:02X}".encode('ascii') + b"\r\n"

def main():
    p = GnssStreamParser()

    print("--- TEST NMEA OK ---")
    nmea_ok = _nmea_build("GPGGA,1234,5678")
    for b in nmea_ok:
        typ, msg = p.feed(b)
        if typ != GnssParseResult.PROCESSING:
            print(f"TYPE: {typ}, MSG: {msg}")

    print("--- TEST NMEA CHKSUM ERROR ---")
    nmea_bad = bytearray(nmea_ok)
    if len(nmea_bad) >= 4 and nmea_bad[-4] in b"0123456789ABCDEF":
        nmea_bad[-4] = (nmea_bad[-4] + 1) if nmea_bad[-4] != ord('F') else ord('0')
    for b in bytes(nmea_bad):
        typ, msg = p.feed(b)
        if typ != GnssParseResult.PROCESSING:
            print(f"TYPE: {typ}, MSG: {msg}")

    print("--- TEST UBX OK ---")
    payload = [0xAA, 0xBB, 0xCC, 0xDD]
    header  = [0x01, 0x02, 0x04, 0x00]
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
    mix2 = b"abc" + ubx + b"def" + b"$"
    for b in mix2:
        typ, msg = p.feed(b)
        if typ != GnssParseResult.PROCESSING:
            print(f"TYPE: {typ}, MSG: {msg}")

if __name__ == "__main__":
    main()
