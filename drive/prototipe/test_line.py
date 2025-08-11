import serial

ser = serial.Serial(
    port='/dev/howerboard',
    baudrate=115200,
    timeout=1,
    dsrdtr=False,   # nepoužívat DTR
    rtscts=False    # nepoužívat RTS/CTS
)

ser.setDTR(False)  # explicitně vypni DTR
ser.setRTS(False)  # explicitně vypni RTS

ser.write(b'PING\n')
response = ser.readline()
print(response.decode())

ser.close()
