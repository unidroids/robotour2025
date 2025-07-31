import socket
import threading
import serial
import time

HOST = '127.0.0.1'
PORT = 9003

SERIAL_PORT = '/dev/howerboard'   # Přizpůsob podle skutečnosti!
SERIAL_BAUD = 115200

serial_lock = threading.Lock()
serial_conn = None

def open_serial():
    global serial_conn
    if serial_conn is None or not serial_conn.is_open:
        serial_conn = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
        print(f"Serial otevřen na {SERIAL_PORT}")
    else:
        print("Serial už je otevřený.")

def close_serial():
    global serial_conn
    if serial_conn and serial_conn.is_open:
        serial_conn.close()
        print("Serial uzavřen.")
    else
        serial_conn.close()
        print("Not opened.")

def send_serial(data: bytes):
    global serial_conn
    with serial_lock:
        if serial_conn and serial_conn.is_open:
            serial_conn.write(data)
            print(f"Odesláno na serial: {data.hex()}")
        else:
            print("Serial není otevřen.")

def handle_client(conn, addr):
    print(f"📡 Klient připojen: {addr}")
    global serial_conn

    def send(msg):
        conn.sendall((msg + '\n').encode())

    try:
        with conn:
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                cmd_line = data.decode().strip()
                parts = cmd_line.split()
                if not parts:
                    continue
                cmd = parts[0].upper()

                if cmd == "PING":
                    send("PONG")

                elif cmd == "START":
                    open_serial()
                    send("OK")

                elif cmd == "STOP":
                    close_serial()
                    send("OK")

                elif cmd == "PWM":
                    if len(parts) == 3:
                        try:
                            pwm_left = int(parts[1])
                            pwm_right = int(parts[2])
                            # Zde zvol vlastní formát odesílaných dat!
                            # Příklad: 2x int16 jako little endian
                            data_bytes = pwm_left.to_bytes(2, 'little', signed=True) + pwm_right.to_bytes(2, 'little', signed=True)
                            send_serial(data_bytes)
                            send("OK")
                        except Exception as e:
                            send(f"ERR {e}")
                    else:
                        send("ERR Param count")

                elif cmd == "BREAK":
                    # Př. break jako dvě nuly:
                    data_bytes = (0).to_bytes(2, 'little', signed=True) + (0).to_bytes(2, 'little', signed=True)
                    send_serial(data_bytes)
                    send("OK")

                elif cmd == "EXIT":
                    send("BYE")
                    break

                else:
                    send("ERR Unknown cmd")

    except Exception as e:
        print(f"Chyba: {e}")
    finally:
        conn.close()
        print(f"🔌 Odpojeno: {addr}")

def start_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT))
    server.listen()
    print(f"🚦 robot-hoverboard server naslouchá na {HOST}:{PORT}")
    try:
        while True:
            conn, addr = server.accept()
            thread = threading.Thread(target=handle_client, args=(conn, addr))
            thread.start()
    finally:
        server.close()
        close_serial()
        print("🛑 Server ukončen.")

if __name__ == "__main__":
    start_server()
