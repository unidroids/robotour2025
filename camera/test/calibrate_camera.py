
import cv2
import numpy as np
import os

# Parametry šachovnice (vnitřní rohy)
CHESSBOARD_SIZE = (9, 6)
SQUARE_SIZE = 1.0  # Může být v cm nebo mm, ale musí být jednotné

# Kritéria pro ukončení hledání rohů
CRITERIA = (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001)

objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE

objpoints = []  # 3D body v reálném světě
imgpoints = []  # 2D body v obraze

def gst_pipeline(sensor_id=0, w=540, h=480, fps=10):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={w}, height={h}, framerate={fps}/1 ! "
        "nvvidconv ! video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink drop=true"
    )


cap = cv2.VideoCapture(gst_pipeline(0), cv2.CAP_GSTREAMER)
#cap = cv2.VideoCapture(0)  # nebo pipeline pro Jetson

print("Stiskněte 's' pro uložení rohů, 'q' pro ukončení a kalibraci.")
while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret_corners, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

    if ret_corners:
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), CRITERIA)
        cv2.drawChessboardCorners(frame, CHESSBOARD_SIZE, corners2, ret_corners)

    cv2.imshow("Kalibrace", frame)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('s') and ret_corners:
        print("Uloženo.")
        objpoints.append(objp)
        imgpoints.append(corners2)
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

if len(objpoints) < 3:
    print("Nedostatek snímků pro kalibraci.")
    exit()

ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("Kamera kalibrována.")
print("cameraMatrix:\n", cameraMatrix)
print("distCoeffs:\n", distCoeffs)

np.savez("camera_calibration.npz",
         cameraMatrix=cameraMatrix,
         distCoeffs=distCoeffs)
print("Uloženo do camera_calibration.npz")
