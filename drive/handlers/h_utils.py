from typing import Tuple

__all__ = [
    "parse_message",
]

# --- util: parse celé NMEA-like věty b"$KOD<payload>*CS\r\n" -> (code,[fields]) ---
def parse_message(msg: bytes) -> Tuple[str, list]:
    """
    Vrací (code and values).
    Vyhazuje ValueError, pokud chybí $, * nebo nesedí CS.
    """

    msg_code =  msg[1:4].decode(errors="ignore")
    payload = msg[4:-5].decode(errors="ignore")
    fields = payload.split(",")
    return msg_code, fields