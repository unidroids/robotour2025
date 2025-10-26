"""parser.py – inkrementální parser pro příchozí věty z hoverboardu

Formát věty (ASCII):
  "$XXX<value>,<value>,<value>,<value>*\r\n"
  - $   : úvodní znak
  - XXX : kód zprávy, 3× velké písmeno (IAM, INM, MSM, ODM, DIM, SEM, SWM)
  - <value>: celočíselné hodnoty (mohou být i se znaménkem), vždy přesně 4, oddělené čárkou
  - *   : ukončovací znak hodnot
  - CRLF: konec věty ("\r\n")

Cíl:
  - rychlý, inkrementální parser vhodný pro 921600 Bd
  - validace struktury před předáním dál (FIFO)
  - žádné blokující I/O; bez printů v hot path

Použití:
  parser = DriveParser()
  messages = parser.feed(b"$IAM1,2,3,4*\r\n...")
  for m in messages:
      # vložit do příchozí FIFO

Poznámky k výkonu:
  - používá interní bytestream buffer a hledá CRLF\n
  - regex je předkompilovaný (rychlý path)
  - ořezává šum před "$" a chrání se proti nekonečné větě (max_line)
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple
import re
import time

__all__ = [
    "DriveRx",
    "DriveParser",
    "VALID_CODES",
]

# Povolené kódy zpráv
VALID_CODES = {"IAM", "INM", "MSM", "ODM", "DIM", "SEM", "SWM"}

# Předkompilovaný regex přesnou strukturou věty
# Příklad: $IAM-1,0,25,123*\r\n
_RX = re.compile(
    rb"^\$(?P<code>[A-Z]{3})(?P<vals>-?\d+,-?\d+,-?\d+,-?\d+)\*\r\n$"
)

@dataclass(frozen=True)
class DriveRx:
    code: str
    values: Tuple[int, int, int, int]
    raw: bytes  # včetně CRLF
    t_mono: float  # čas dekódování (monotonic)

    def __repr__(self) -> str:
        v = ",".join(str(x) for x in self.values)
        return f"DriveRx(code={self.code}, values=({v}))"


class DriveParser:
    """Inkrementální parser řádkovaných vět.

    feed() lze volat s libovolně dlouhým chunkem bytes. Vrací seznam již
    kompletně validovaných zpráv DriveRx. V případě poškozených dat se
    parser pokusí resynchronizovat na další '\r\n' + '$'.
    """

    def __init__(self, *, max_line: int = 128):
        self._buf = bytearray()
        self._max_line = max_line
        # Statistiky (nejsou povinné; čitelnost při ladění)
        self.bad_lines = 0
        self.too_long_lines = 0
        self.unknown_codes = 0

    def reset(self) -> None:
        """Vyčistí interní buffer a statistiky (bez side‑effectů)."""
        self._buf.clear()
        self.bad_lines = 0
        self.too_long_lines = 0
        self.unknown_codes = 0

    def feed(self, chunk: bytes) -> List[DriveRx]:
        if not chunk:
            return []
        self._buf.extend(chunk)
        out: List[DriveRx] = []

        while True:
            # Najdi konec řádku (CRLF)
            nl = self._find_eol(self._buf)
            if nl < 0:
                # Pokud je buffer příliš dlouhý bez CRLF, odhoď vše před posledním '$'
                if len(self._buf) > self._max_line:
                    self._drop_until_last_dollar()
                break

            # Máme celý řádek včetně CRLF
            line = bytes(self._buf[: nl + 2])  # vč. CRLF
            del self._buf[: nl + 2]

            # Ořezat šum před '$'
            start = line.find(b"$")
            if start > 0:
                line = line[start:]

            # Rychlá validace délky
            if len(line) > self._max_line:
                self.too_long_lines += 1
                continue

            # Musí končit CRLF
            if not line.endswith(b"\r\n"):
                self.bad_lines += 1
                continue

            # Pustit regex
            m = _RX.match(line)
            if not m:
                self.bad_lines += 1
                continue

            code = m.group("code").decode("ascii")
            if code not in VALID_CODES:
                self.unknown_codes += 1
                continue

            vals_b = m.group("vals")  # b"v1,v2,v3,v4"
            try:
                v1, v2, v3, v4 = (int(x) for x in vals_b.split(b","))
            except ValueError:
                self.bad_lines += 1
                continue

            out.append(
                DriveRx(
                    code=code,
                    values=(v1, v2, v3, v4),
                    raw=line,
                    t_mono=time.monotonic(),
                )
            )

        return out

    # --- pomocné metody ---
    @staticmethod
    def _find_eol(buf: bytearray) -> int:
        """Najde index '\r' tak, aby následovalo '\n'. Pokud nenalezeno, vrací -1."""
        # Hledáme od konce (typicky méně kroků)
        idx = buf.rfind(b"\n")
        if idx <= 0:
            return -1
        # zajisti že před \n je \r
        if idx > 0 and buf[idx - 1] == 13:  # ord('\r')
            return idx - 1
        return -1

    def _drop_until_last_dollar(self) -> None:
        """Při overflow vyhoď vše do posledního '$' (resync)."""
        last = self._buf.rfind(b"$")
        if last <= 0:
            # nic rozumného – čistý reset
            self._buf.clear()
        else:
            del self._buf[: last]


# --- jednoduchý self‑test ---
if __name__ == "__main__":
    p = DriveParser()
    sample = (
        b"garbage$IAM1,2,3,4*\r\n$MSM-1,0,250,9*\r\n$BAD1,2,3,4*\r\n$ODM0,0,0,0*\r\n"
    )
    msgs = p.feed(sample)
    print("decoded:", msgs)
    print("bad:", p.bad_lines, "too_long:", p.too_long_lines, "unknown:", p.unknown_codes)
