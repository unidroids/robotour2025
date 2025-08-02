# calibrate_view.py (LAPTOP)
import socket
import struct
import numpy as np
import cv2

PORT = 5010
HOST = "0.0.0.0"

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(1)
conn, _ = server.accept()
print("Připojeno, čekám na obrázky...")

while True:
    length = conn.recv(4)
    if not length:
        break
    size = struct.unpack(">I", length)[0]
    data = b''
    while len(data) < size:
        packet = conn.recv(size - len(data))
        if not packet:
            break
        data += packet
    if not data:
        break
    # Přijmi a zobraz snímek
    arr = np.frombuffer(data, dtype=np.uint8)
    img = cv2.imdecode(arr, 1)
    cv2.imshow("Kalibrace – laptop", img)
    key = cv2.waitKey(1) & 0xFF
    # Pošli zpět znak (s=save, q=quit, nic jiného)
    if key == ord('s'):
        conn.sendall(b's')
    elif key == ord('q'):
        conn.sendall(b'q')
        break
    else:
        conn.sendall(b' ')
conn.close()
server.close()
cv2.destroyAllWindows()
