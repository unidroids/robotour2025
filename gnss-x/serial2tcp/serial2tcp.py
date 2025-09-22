import socket
import serial
import threading

DEVICE = '/dev/gnss1'
BAUD = 921600 #115200
TCP_PORT = 5000

def read_serial(ser, conn):
    try:
        while True:
            data = ser.read(1024)
            if data:
                try:
                    conn.sendall(data)
                except Exception:
                    break
    except Exception:
        pass

def write_serial(ser, conn):
    try:
        while True:
            data = conn.recv(1024)
            if not data:
                break
            ser.write(data)
    except Exception:
        pass

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
            ser = serial.Serial(DEVICE, BAUD, timeout=0.1)
            t1 = threading.Thread(target=read_serial, args=(ser, conn), daemon=True)
            t2 = threading.Thread(target=write_serial, args=(ser, conn), daemon=True)
            t1.start()
            t2.start()
            # Počkej na ukončení (dokud se nezavře spojení)
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

if __name__ == '__main__':
    main()
