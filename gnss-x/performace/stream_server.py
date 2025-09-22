import socket
import struct
import json

HOST = "127.0.0.1"
PORT_TEXT = 8001
PORT_BIN = 8002

fmt = "<iiiIIiIiIiii iii"  
# lat, lon, height, hAcc, vAcc, gSpeed, heading, sAcc, cAcc, roll, pitch, zAngRate, xAccel, yAccel, zAccel
# jednoduchý příklad – vše jako int32 (I=uint32, i=int32)

def handle_text(conn):
    buf = b""
    while True:
        data = conn.recv(4096)
        if not data:
            break
        buf += data
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            try:
                record = json.loads(line.decode())
                # pro test rozbalení
                lat, lon, height = record["lat"], record["lon"], record["height"]
            except Exception as e:
                print("JSON error:", e)

def handle_bin(conn):
    size = struct.calcsize(fmt)
    buf = b""
    while True:
        data = conn.recv(4096)
        if not data:
            break
        buf += data
        while len(buf) >= size:
            block, buf = buf[:size], buf[size:]
            try:
                values = struct.unpack(fmt, block)
                lat, lon, height = values[0], values[1], values[2]
            except Exception as e:
                print("Struct error:", e)

def start_server(port, handler):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, port))
        s.listen()
        print(f"Server listening on {HOST}:{port}")
        conn, addr = s.accept()
        with conn:
            print("Connected by", addr)
            handler(conn)

if __name__ == "__main__":
    # spusť paralelně dvě instance – jednu pro text, jednu pro bin
    import sys
    mode = sys.argv[1] if len(sys.argv) > 1 else "text"
    if mode == "text":
        start_server(PORT_TEXT, handle_text)
    else:
        start_server(PORT_BIN, handle_bin)
