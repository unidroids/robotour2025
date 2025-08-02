# jetson_send.py
import socket
import cv2

IP_LAPTOP = "192.168.55.100"
PORT = 5000

def gst_pipeline(sensor_id=0, w=1640, h=1232, fps=10):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={w}, height={h}, framerate={fps}/1 ! "
        "nvvidconv ! video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink drop=true"
    )

cap = cv2.VideoCapture(gst_pipeline(0), cv2.CAP_GSTREAMER)


#cap = cv2.VideoCapture(0)  # nebo pipeline pro Jetson

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_LAPTOP, PORT))
print("Connected to laptop!")

while True:
    ret, frame = cap.read()
    if not ret:
        continue
    # Zmenšit/komprimovat
    _, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    data = buf.tobytes()
    # Pošli délku + data (4 bajty délka)
    s.sendall(len(data).to_bytes(4, 'big') + data)
    # pauza (10fps)
    if cv2.waitKey(1) == 27:
        break

cap.release()
s.close()
