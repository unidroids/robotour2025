import socket
import struct
import json
import time
import random

HOST = "127.0.0.1"
PORT_TEXT = 8001
PORT_BIN = 8002

fmt = "<iiiIIiIiIiii iii"

def make_data():
    return {
        "lat": int(50_0000000 + random.randint(-1000, 1000)),
        "lon": int(14_0000000 + random.randint(-1000, 1000)),
        "height": 260000,
        "hAcc": 50,
        "vAcc": 80,
        "gSpeed": 1234,
        "heading": 900000,
        "sAcc": 20,
        "cAcc": 50,
        "roll": 10,
        "pitch": -5,
        "zAngRate": 100,
        "xAccel": 5,
        "yAccel": 0,
        "zAccel": -9,
    }

def send_text():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT_TEXT))
        for _ in range(10000):
            msg = json.dumps(make_data()).encode() + b"\n"
            s.sendall(msg)

def send_bin():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT_BIN))
        for _ in range(10000):
            d = make_data()
            msg = struct.pack(
                fmt,
                d["lat"], d["lon"], d["height"], d["hAcc"], d["vAcc"],
                d["gSpeed"], d["heading"], d["sAcc"], d["cAcc"],
                d["roll"], d["pitch"], d["zAngRate"],
                d["xAccel"], d["yAccel"], d["zAccel"]
            )
            s.sendall(msg)

if __name__ == "__main__":
    import sys
    mode = sys.argv[1] if len(sys.argv) > 1 else "text"
    start = time.time()
    if mode == "text":
        send_text()
    else:
        send_bin()
    print(f"Finished in {time.time() - start:.3f}s")
