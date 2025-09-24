# gnss/handlers/nmea_gga_handler.py

class NmeaGgaHandler:
    """
    Handler, který ukládá poslední GGA větu (neparsovanou) do kontextu služby.
    """
    def __init__(self, context):
        # context = objekt GNSS služby nebo cokoliv s ._last_gga
        self._last_gga = None

    def handle(self, nmea_text):
        """
        Zavolej tuto metodu pro každou příchozí GGA větu (text)
        """
        # Ověř, že jde o GGA zprávu (bezpečnost)
        if nmea_text.startswith("$GNGGA") or nmea_text.startswith("$GPGGA"):
            self._last_gga = nmea_text
            # volitelně: log, případně další akce
            # print(f"[GGA handler] Uložena GGA: {nmea_text.strip()}")

    def get_gga(self):
        return self._last_gga