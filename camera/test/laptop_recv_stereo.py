
import socket
import cv2
import pickle
import struct

HOST = '0.0.0.0'
PORT = 5001

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)

conn, addr = server_socket.accept()
data = b""
payload_size = struct.calcsize("L")

while True:
    while len(data) < payload_size:
        packet = conn.recv(4096)
        if not packet:
            break
        data += packet
    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack("L", packed_msg_size)[0]

    while len(data) < msg_size:
        data += conn.recv(4096)
    frame_data = data[:msg_size]
    data = data[msg_size:]

    frame = pickle.loads(frame_data)
    h, w = 720, 1280
    scale = min(w / frame.shape[1], h / frame.shape[0])
    frame = cv2.resize(frame, (int(frame.shape[1]*scale), int(frame.shape[0]*scale)))
    cv2.imshow("Stereo Image", frame)
    if cv2.waitKey(1) == 27:
        break

conn.close()
server_socket.close()
