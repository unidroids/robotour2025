import socket
import serial
import threading
import os

DEVICE = '/dev/gnss1'
BAUD = 921600
TCP_PORT = 5000

# Získání cesty ke skriptu:
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
SERIAL_TO_TCP_LOG = os.path.join(BASE_DIR, "serial_to_tcp.log")
TCP_TO_SERIAL_LOG = os.path.join(BASE_DIR, "tcp_to_serial.log")

def read_serial(ser, conn, log_file_handle):
    try:
        while True:
            data = ser.read(1)
            if ser.in_waiting:
                data += ser.read(ser.in_waiting)
            if data:
                try:
                    conn.sendall(data)
                    log_file_handle.write(data)
                    log_file_handle.flush()
                except Exception:
                    break
    except Exception as e:
        print("read_serial exception:", e)

def write_serial(ser, conn, log_file_handle):
    try:
        while True:
            data = conn.recv(1024)
            if not data:
                break
            ser.write(data)
            log_file_handle.write(data)
            log_file_handle.flush()
    except Exception as e:
        print("write_serial exception:", e)

def main():
    print(f"Listening on 0.0.0.0:{TCP_PORT}, streaming {DEVICE}...")

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', TCP_PORT))
    server.listen(1)

    while True:
        conn, addr = server.accept()
        print(f"Accepted connection from {addr}")

        try:
            # Log soubory OTEVŘÍT v režimu 'wb' (přepis) při každém novém spojení:
            serial_to_tcp_log = open(SERIAL_TO_TCP_LOG, "wb")
            tcp_to_serial_log = open(TCP_TO_SERIAL_LOG, "wb")

            ser = serial.Serial(DEVICE, BAUD, timeout=0.05)

            t1 = threading.Thread(
                target=read_serial, args=(ser, conn, serial_to_tcp_log), daemon=True)
            t2 = threading.Thread(
                target=write_serial, args=(ser, conn, tcp_to_serial_log), daemon=True)
            t1.start()
            t2.start()
            t1.join()
            t2.join()
        except Exception as e:
            print("Connection closed:", e)
        finally:
            try:
                conn.close()
            except:
                pass
            try:
                ser.close()
            except:
                pass
            try:
                serial_to_tcp_log.close()
            except:
                pass
            try:
                tcp_to_serial_log.close()
            except:
                pass

if __name__ == '__main__':
    main()
