import serial
import threading

SERIAL_PORT = '/dev/howerboard'
SERIAL_BAUD = 115200

serial_lock = threading.Lock()
serial_conn = None

def open_serial():
    global serial_conn
    if serial_conn is None or not serial_conn.is_open:
        serial_conn = serial.Serial(
            port=SERIAL_PORT,
            baudrate=SERIAL_BAUD,
            timeout=1,
            dsrdtr=False,   # nepoužívat DTR
            rtscts=False    # nepoužívat RTS/CTS
        )
        ser.setDTR(False)  # explicitně vypni DTR
        ser.setRTS(False)  # explicitně vypni RTS        
        print(f"Serial otevřen na {SERIAL_PORT}")
    else:
        print("Serial už je otevřený.")

def close_serial():
    global serial_conn
    if serial_conn and serial_conn.is_open:
        serial_conn.close()
        print("Serial uzavřen.")
    else:
        print("Serial není otevřen.")

def send_serial(data: bytes):
    global serial_conn
    with serial_lock:
        if serial_conn and serial_conn.is_open:
            serial_conn.write(data)
            print(f"Odesláno na serial: {data.hex()}")
        else:
            print("Serial není otevřen.")
