__all__ = [
    "parse_message",
]

# --- util: parse celé NMEA-like věty b"$KOD<payload>*CS\r\n" -> (code,[fields]) ---
def parse_message(msg: bytes) -> Tuple[str, list]:
    """
    Vrací (code and values).
    Vyhazuje ValueError, pokud chybí $, * nebo nesedí CS.
    """

    msg_code =  msg[1:4].decode(errors="ignore").
    payload = msg[5:-5].decode(errors="ignore").strip()
    fields = payload.split(",")
    return msg_code, fields