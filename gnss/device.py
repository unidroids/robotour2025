# device.py
import serial
import threading
from ubx import build_msg

class GNSSDevice:
    def __init__(self):
        self.port = None
        self.lock = threading.Lock()
        self.running = False
        self.device_type = None  # F9R nebo D9S

    def start(self):
        try:
            # testujeme gnss1/gnss2
            for candidate in ["/dev/gnss1", "/dev/gnss2"]:
                try:
                    ser = serial.Serial(candidate, baudrate=38400, timeout=1)
                    ser.write(build_msg(0x0A, 0x04))  # MON-VER
                    data = ser.read(200)
                    if b"F9R" in data:
                        self.device_type = "F9R"
                        self.port = ser
                        break
                    elif b"D9S" in data:
                        self.device_type = "D9S"
                        self.port = ser
                        break
                    ser.close()
                except Exception:
                    continue
            if not self.port:
                return False
            self.running = True
            return True
        except Exception as e:
            print(f"Chyba startu GNSS: {e}")
            return False

    def get_state(self):
        if not self.running:
            return "IDLE"
        # TODO: dotaz na UBX NAV-SAT (satellites, fix)
        return f"RUNNING {self.device_type} sat=?? fix=?"

    def calibrate(self):
        if not self.running:
            return "ERROR not running"
        # TODO: poslat UBX zpravy pro kalibraci IMU (jen F9R)
        return "CALIBRATION not implemented"

    def get_fix(self):
        if not self.running:
            return "ERROR not running"
        # TODO: dotaz na UBX NAV-PVT
        return "lat=?? lon=?? alt=?? heading=?? speed=?? time=??"

gnss_device = GNSSDevice()
