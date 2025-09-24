import socket
import time
import os

def extract_gga(data):
    # Procházej všechna $... v odpovědi
    lines = data.split('$')
    for line in lines:
        # GGA může být GNGGA nebo GPGGA, případně i GLGGA...
        if line.startswith("GNGGA") or line.startswith("GPGGA"):
            # Najdi konec věty (většinou '\r\n', někdy jen '\r' nebo '\n')
            for end in ['\r\n', '\r', '\n']:
                if end in line:
                    return '$' + line.split(end)[0]
            # Kdyby tam nebyl konec, vrať celou větu až do konce pole
            return '$' + line
    return None

HOST = '127.0.0.1'
PORT = 5000
body = "EIGNQ,GGA"
def nmea_checksum(sentence):
    cksum = 0
    for c in sentence:
        cksum ^= ord(c)
    return f"{cksum:02X}"
msg = f"${body}*{nmea_checksum(body)}\r\n"

filename = os.path.join(os.path.dirname(__file__), "gga_log.txt")

with socket.create_connection((HOST, PORT), timeout=2) as sock:
    sock.sendall(msg.encode("ascii"))
    time.sleep(0.2)
    resp = sock.recv(4096).decode(errors='ignore')
    with open(filename, "w", encoding="utf-8") as f:
        f.write(resp)
    gga = extract_gga(resp)
    if gga:
        print('GGA:', gga)
    else:
        print('GGA nenačtena (žádná odpověď neobsahuje GGA)')
