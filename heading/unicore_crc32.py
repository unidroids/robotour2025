from __future__ import annotations
from typing import Union

Buffer = Union[bytes, bytearray, memoryview]

__all__ = ["UnicoreCRC32"]

class UnicoreCRC32:
    """
    CRC32 podle Unicore (N4 / UM982 manuál):
    - poly:   0x04C11DB7 (reflektovaně 0xEDB88320)
    - init:   0x00000000
    - xorout: 0x00000000
    - refin:  True
    - refout: True
    """

    POLY = 0xEDB88320  # reflektovaný polynom
    _TABLE: tuple[int, ...] | None = None

    CRC_HEX_LEN = 8
    SUFFIX_LEN = 2       # \r\n
    STAR_LEN = 1         # '*'
    STAR_OFFSET_FROM_END = CRC_HEX_LEN + SUFFIX_LEN + STAR_LEN  # 11

    @classmethod
    def _make_table(cls) -> tuple[int, ...]:
        table = []
        for i in range(256):
            crc = i
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ cls.POLY
                else:
                    crc >>= 1
            table.append(crc & 0xFFFFFFFF)
        return tuple(table)

    @classmethod
    def table(cls) -> tuple[int, ...]:
        if cls._TABLE is None:
            cls._TABLE = cls._make_table()
        return cls._TABLE

    def __init__(self, initial: int = 0):
        self._crc = initial & 0xFFFFFFFF

    def update(self, data: Buffer) -> None:
        """Inkrementální update nad bytes/bytearray/memoryview."""
        table = self.table()
        crc = self._crc
        for b in memoryview(data):
            crc = table[(crc ^ b) & 0xFF] ^ (crc >> 8)
        self._crc = crc & 0xFFFFFFFF

    def digest(self) -> int:
        return self._crc & 0xFFFFFFFF

    def hexdigest(self) -> str:
        return f"{self.digest():08x}"

    # --- jednorázový výpočet nad libovolným bufferem ---

    @classmethod
    def compute(cls, data: Buffer) -> int:
        c = cls()
        c.update(data)
        return c.digest()

    # --- práce s ASCII rámcem z UARTu: #payload*CCCCCCCC\r\n ---

    @classmethod
    def _as_mv(cls, frame: Buffer) -> memoryview:
        return memoryview(frame)

    @classmethod
    def compute_ascii_frame(cls, frame: Buffer) -> int:
        """
        Spočítá CRC payloadu pro ASCII rámec:
        frame = b"#" + payload + b"*" + 8×hex + b"\\r\\n"
        CRC se počítá z payloadu (mezi '#' a '*').
        """
        mv = cls._as_mv(frame)

        if len(mv) < 1 + cls.STAR_OFFSET_FROM_END:
            raise ValueError("Rámec je příliš krátký")

        # začátek '#'
        if mv[0] != 0x23:  # '#'
            raise ValueError("Rámec musí začínat znakem '#'")

        # konec '\r\n'
        if mv[-2] != 0x0D or mv[-1] != 0x0A:
            raise ValueError("Rámec musí končit '\\r\\n'")

        # pozice '*'
        star_index = len(mv) - cls.STAR_OFFSET_FROM_END
        if mv[star_index] != 0x2A:  # '*'
            raise ValueError("Na očekávané pozici od konce není '*'")

        # payload mezi '#' a '*'
        payload = mv[1:star_index]
        return cls.compute(payload)

    @classmethod
    def extract_crc_from_frame(cls, frame: Buffer) -> int:
        """
        Vybere očekávané CRC z ASCII rámce:
        b"#" + payload + b"*" + b"%08x" + b"\\r\\n"
        """
        mv = cls._as_mv(frame)

        if len(mv) < 1 + cls.STAR_OFFSET_FROM_END:
            raise ValueError("Rámec je příliš krátký")

        star_index = len(mv) - cls.STAR_OFFSET_FROM_END
        crc_bytes = bytes(mv[star_index + 1 : -cls.SUFFIX_LEN])  # 8 hex znaků

        if len(crc_bytes) != cls.CRC_HEX_LEN:
            raise ValueError("CRC část nemá 8 znaků")

        try:
            return int(crc_bytes.decode("ascii"), 16)
        except ValueError as e:
            raise ValueError("CRC část není validní hex") from e

    @classmethod
    def verify_ascii_frame(cls, frame: Buffer) -> bool:
        """Ověří celý ASCII rámec včetně CRC a CRLF."""
        computed = cls.compute_ascii_frame(frame)
        expected = cls.extract_crc_from_frame(frame)
        return (computed & 0xFFFFFFFF) == (expected & 0xFFFFFFFF)


# --- jednoduchý self-test se vzorovými větami z manuálu ---

if __name__ == "__main__":
    frame1 = (
        b'#VERSIONA,79,GPS,FINE,2326,378237000,15434,0,18,889;'
        b'"UM982","R4.10Build15434","HRPT00-S10C-P",'
        b'"2310415000012-LR23A2225208904","ff2740966a10124c","2024/08/08"*769fd54f\r\n'
    )

    frame2 = b'#OBSVHA,97,GPS,FINE,2190,359897000,0,0,18,14;0*9d38304c\r\n'
    frame3 = b'#UNIHEADINGA,92,GPS,FINE,2392,517944600,0,0,18,6;INSUFFICIENT_OBS,NONE,0.0000,0.0000,0.0000,0.0000,0.0000,0.0000,"",0,0,0,0,0,00,0,0*044457ef\r\n'

    # výpočet z payloadu (mezi '#' a '*')
    crc1 = UnicoreCRC32.compute_ascii_frame(frame1)
    crc2 = UnicoreCRC32.compute_ascii_frame(frame2)
    

    print("frame1 CRC:", hex(crc1))
    print("frame2 CRC:", hex(crc2))

    assert crc1 == 0x769fd54f
    assert crc2 == 0x9d38304c
    assert UnicoreCRC32.verify_ascii_frame(frame1)
    assert UnicoreCRC32.verify_ascii_frame(frame2)
    assert UnicoreCRC32.verify_ascii_frame(frame3)

    print("Všechny testy prošly.")
