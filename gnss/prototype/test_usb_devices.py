import serial

def detect_gnss(port="/dev/gnss1"):
    with serial.Serial(port, 9600, timeout=1) as ser:
        # UBX-MON-VER request
        msg = bytes([0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34])
        ser.write(msg)
        data = ser.read(200)  # odpověď je cca 100+ B
        if b"F9R" in data:
            return "F9R"
        elif b"D9S" in data:
            return "D9S"
        else:
            return "Unknown"

print("/dev/gnss1", detect_gnss("/dev/gnss1"))
print("/dev/gnss2", detect_gnss("/dev/gnss2"))
