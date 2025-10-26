"""handlers/nack.py – handler pro INM (Input NACK Message)

Zatím jen jednoduchý výpis pomocí print, bez dalších side‑effectů.
"""
from __future__ import annotations

from parser import DriveRx

__all__ = ["handle"]


def handle(msg: DriveRx) -> None:
    """Zpracuje NACK zprávu (INM)."""
    if msg.code != "INM":
        print(f"[NACK] Unexpected code {msg.code}: {msg.values}")
        return

    v1, v2, v3, v4 = msg.values
    print(f"[NACK] INM  p1={v1} p2={v2} p3={v3} p4={v4}  t_mono={msg.t_mono:.6f}")


if __name__ == "__main__":
    # Mini self‑test
    import time
    m = DriveRx(code="INM", values=(10, 0, 4, 99), raw=b"$INM10,0,4,99*\r\n", t_mono=time.monotonic())
    handle(m)
