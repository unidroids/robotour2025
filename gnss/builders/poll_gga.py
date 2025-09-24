# gnss/builders/poll_gga.py

def build_gnq_gga_poll():
    """
    Vytvoří NMEA GNQ poll request na GGA (univerzální pro multi-GNSS u-blox).
    Viz UBX-22010984, NMEA-Standard-GNQ.
    Výsledek: b'$EIGNQ,GGA*22\\r\\n'
    """
    def nmea_checksum(sentence):
        cksum = 0
        for c in sentence:
            cksum ^= ord(c)
        return f"{cksum:02X}"

    body = "EIGNQ,GGA"
    msg = f"${body}*{nmea_checksum(body)}\r\n"
    return msg.encode("ascii")  # Vratíme binárně, lze rovnou poslat na socket

# Pokud by někdy bylo třeba GPQ poll (GPS only), použij:
def build_gpq_gga_poll():
    body = "EIGPQ,GGA"
    def nmea_checksum(sentence):
        cksum = 0
        for c in sentence:
            cksum ^= ord(c)
        return f"{cksum:02X}"
    msg = f"${body}*{nmea_checksum(body)}\r\n"
    return msg.encode("ascii")
