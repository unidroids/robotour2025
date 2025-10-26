"""handlers/ack.py – handler pro IAM (Input ACK Message)

Zatím jen jednoduchý výpis pomocí print, bez dalších side‑effectů.
"""
from __future__ import annotations

from parser import DriveRx

__all__ = ["handle"]


def handle(msg: DriveRx) -> None:
    """Zpracuje ACK zprávu (IAM)."""
    # Ochrana proti omylem volanému handleru
    if msg.code != "IAM":
        print(f"[ACK] Unexpected code {msg.code}: {msg.values}")
        return

    v1, v2, v3, v4 = msg.values
    # Výpis (zatím bez interpretace významu polí)
    print(f"[ACK] IAM  p1={v1} p2={v2} p3={v3} p4={v4}  t_mono={msg.t_mono:.6f}")


if __name__ == "__main__":
    # Mini self‑test
    import time
    m = DriveRx(code="IAM", values=(1, 2, 3, 4), raw=b"$IAM1,2,3,4*\r\n", t_mono=time.monotonic())
    handle(m)
