import struct

class NavAttHandler:
    def __init__(self):
        self._last_print_sec = None

    def handle(self, msg_class, msg_id, payload):
        if len(payload) < 32:
            print("[NAV-ATT] Wrong payload length:", len(payload))
            return

        iTOW, version = struct.unpack('<IB', payload[:5])
        # 3B reserved, přeskočíme
        roll, pitch, heading = struct.unpack('<iii', payload[8:20])
        accRoll, accPitch, accHeading = struct.unpack('<III', payload[20:32])

        sec = iTOW // 1000
        if self._last_print_sec is None or sec != self._last_print_sec:
            self._last_print_sec = sec
            print(
                f"[NAV-ATT] iTOW={iTOW} roll={roll/1e5:.3f}° pitch={pitch/1e5:.3f}° heading={heading/1e5:.3f}° "
                f"accRoll={accRoll/1e5:.3f} accPitch={accPitch/1e5:.3f} accHeading={accHeading/1e5:.3f}"
            )
