# gnss/handlers/nmea_gga_handler.py

class NmeaGgaHandler:
    """
    Handler, který ukládá poslední GGA větu (neparsovanou) do kontextu služby.
    """
    def __init__(self):
        self._last_gga = None

    def handle(self, sentence):
        """
        Zavolej tuto metodu pro každou příchozí GGA větu (text)
        """
        self._last_gga = sentence
        print(f"[NMEA-GGA] {sentence.decode().strip()}")

    def get_last_gga(self):
        return self._last_gga