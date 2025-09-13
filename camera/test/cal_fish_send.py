# calibrate_send_fisheye.py (JETSON)
import cv2
import numpy as np
import socket
import struct
from pyzbar import pyzbar

import time

PORT = 5010
LAPTOP_IP = "192.168.55.100"

def gst_pipeline(sensor_id=0, w=1000, h=800, fps=10):
    base = (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={w}, height={h}, framerate={fps}/1 ! "
        "nvvidconv flip-method=3 ! video/x-raw,format=BGRx ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false"
    )
    return base

t0 = time.time()
cap = cv2.VideoCapture(gst_pipeline(1), cv2.CAP_GSTREAMER)
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((LAPTOP_IP, PORT))

CHESSBOARD_SIZE = (9, 6)
CRITERIA = (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001)
objp = np.zeros((1, CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)

objpoints, imgpoints = [], []

# # Předpokládej známé rozlišení (např. z kamery)
w, h = 800, 800

d = np.load("camera_fisheye_calib.npz")
K, D = d["K"], d["D"]
Knew = K.copy()
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), Knew, (w, h), cv2.CV_16SC2)

# Knew[1,2] += 0  # posun cx doprava (výřez posuneš doprava)
# Knew[0,2] += 0  # posun cx doprava (výřez posuneš doprava)
# # nebo Knew[1,2] += ...  pro posun ve svislém směru

# # Nebo můžeš „zoomovat“ (zvětšit ohnisko)
# Knew[0,0] *= 0.8  # fx, horizontální zoom
# Knew[1,1] *= 0.8  # fy, vertikální zoom
# map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), Knew, (w, h), cv2.CV_16SC2)

#print(K)

qr_cnt=0

print("header", time.time() - t0)

while True:
    t0 = time.time()
    ret, frame = cap.read()
    if not ret:
        continue

    print("read", time.time() - t0)
    t0 = time.time()

    frame = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    codes = pyzbar.decode(gray)
    if len(codes) > 0:
        qr_cnt+=1
        print(qr_cnt, codes[0])    

    ret_corners, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)
    show = frame.copy()
    if ret_corners:
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), CRITERIA)
        cv2.drawChessboardCorners(show, CHESSBOARD_SIZE, corners2, ret_corners)

    print("corners", time.time() - t0)
    t0 = time.time()

    # Pošli obrázek (jpg)
    scale = 1
    h, w = show.shape[:2]
    resized = cv2.resize(show, (int(w * scale), int(h * scale)),
                        interpolation=cv2.INTER_AREA)    
    _, buf = cv2.imencode('.jpg', resized, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
    data = buf.tobytes()

    print("resize", time.time() - t0)
    t0 = time.time()

    client.sendall(struct.pack(">I", len(data)) + data)

    print("send", time.time() - t0)
    t0 = time.time()

    # Čekej na odpověď (1 byte)
    resp = client.recv(1)
    if not resp:
        break
    key = resp.decode("utf-8")
    if key == 's' and ret_corners:
        print("Uloženo.")
        objpoints.append(objp)
        imgpoints.append(corners2.reshape(1, -1, 2))
    elif key == 'q':
        print("Konec.")
        break

    print("response", time.time() - t0)
    t0 = time.time()



cap.release()
client.close()

if len(objpoints) < 3:
    print("Nedostatek snímků pro kalibraci.")
    exit()

# --- FISHEYE KALIBRACE ---
objpoints_np = np.array(objpoints, dtype=np.float32)
imgpoints_np = np.array(imgpoints, dtype=np.float32)
N_OK = len(objpoints_np)
K = np.zeros((3, 3))
D = np.zeros((4, 1))
rvecs = [np.zeros((1,1,3), dtype=np.float64) for i in range(N_OK)]
tvecs = [np.zeros((1,1,3), dtype=np.float64) for i in range(N_OK)]

flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC |
         cv2.fisheye.CALIB_CHECK_COND |
         cv2.fisheye.CALIB_FIX_SKEW)

rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
    objpoints_np, imgpoints_np, gray.shape[::-1], K, D, rvecs, tvecs, flags,
    (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 100, 1e-6)
)

print("Kamera (fisheye) kalibrována.")
print("K:\n", K)
print("D:\n", D)
np.savez("camera_fisheye_calib.npz", K=K, D=D)
print("Uloženo do camera_fisheye_calib.npz")
