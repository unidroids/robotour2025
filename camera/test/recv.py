# laptop_recv.py
import socket
import cv2
import numpy as np

IP_LAPTOP = "0.0.0.0"
PORT = 5000

def resize_to_screen(frame, max_width=1280, max_height=720):
    h, w = frame.shape[:2]
    scale = min(max_width / w, max_height / h, 1.0)  # scale nikdy nezvětšuje
    if scale < 1.0:
        frame = cv2.resize(frame, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
    return frame

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((IP_LAPTOP, PORT))
s.listen(1)
print("Waiting for Jetson...")
conn, addr = s.accept()
print(f"Connected from {addr}")

while True:
    # Přijmi délku (4 bajty)
    length = b''
    while len(length) < 4:
        more = conn.recv(4 - len(length))
        if not more:
            break
        length += more
    if not length:
        break
    size = int.from_bytes(length, 'big')

    # Přijmi samotná data
    data = b''
    while len(data) < size:
        more = conn.recv(size - len(data))
        if not more:
            break
        data += more
    if not data:
        break

    # Dekóduj obrázek a zobraz
    frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), 1)
    frame = resize_to_screen(frame, 1280, 720)
    cv2.imshow('Jetson Camera', frame)
    if cv2.waitKey(1) == 27:
        break

conn.close()
s.close()
cv2.destroyAllWindows()
