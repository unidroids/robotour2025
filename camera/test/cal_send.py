# calibrate_send.py (JETSON)
import cv2
import numpy as np
import socket
import struct

PORT = 5010
LAPTOP_IP = "192.168.55.100"  # zadej IP laptopu

def gst_pipeline(sensor_id=0, w=540, h=480, fps=10):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={w}, height={h}, framerate={fps}/1 ! "
        "nvvidconv ! video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink drop=true"
    )


cap = cv2.VideoCapture(gst_pipeline(0), cv2.CAP_GSTREAMER)
#cap = cv2.VideoCapture(0)  # nebo pipeline pro Jetson

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((LAPTOP_IP, PORT))

CHESSBOARD_SIZE = (9, 6)
CRITERIA = (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001)
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
objpoints, imgpoints = [], []

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret_corners, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)
    show = frame.copy()
    if ret_corners:
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), CRITERIA)
        cv2.drawChessboardCorners(show, CHESSBOARD_SIZE, corners2, ret_corners)

    # Pošli obrázek (jpg)
    _, buf = cv2.imencode('.jpg', show, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
    data = buf.tobytes()
    client.sendall(struct.pack(">I", len(data)) + data)

    # Čekej na odpověď (1 byte)
    resp = client.recv(1)
    if not resp:
        break
    key = resp.decode("utf-8")
    if key == 's' and ret_corners:
        print("Uloženo.")
        objpoints.append(objp)
        imgpoints.append(corners2)
    elif key == 'q':
        print("Konec.")
        break

cap.release()
client.close()

if len(objpoints) < 3:
    print("Nedostatek snímků pro kalibraci.")
    exit()

ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)
print("Kamera kalibrována.")
print("cameraMatrix:\n", cameraMatrix)
print("distCoeffs:\n", distCoeffs)
np.savez("camera_calibration.npz", cameraMatrix=cameraMatrix, distCoeffs=distCoeffs)
print("Uloženo do camera_calibration.npz")
