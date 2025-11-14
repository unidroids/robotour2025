import serial

ser = serial.Serial("/dev/robot-heading", baudrate=921600, timeout=1.0)
for line in ser:
    print(line.decode(errors="ignore").strip())
