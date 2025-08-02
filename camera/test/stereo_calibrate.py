
import cv2
import numpy as np
import glob
import os

# Nastavení vzoru šachovnice (vnitřní rohy)
CHESSBOARD_SIZE = (9, 6)
SQUARE_SIZE = 0.025  # velikost pole v metrech

# Cesty ke složkám se snímky
LEFT_IMAGES_DIR = "calib/left"
RIGHT_IMAGES_DIR = "calib/right"

def load_images(folder):
    return sorted(glob.glob(os.path.join(folder, '*.jpg')))

def calibrate_stereo(left_imgs, right_imgs):
    objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    objpoints = []
    imgpoints_left = []
    imgpoints_right = []

    for left_path, right_path in zip(left_imgs, right_imgs):
        imgL = cv2.imread(left_path)
        imgR = cv2.imread(right_path)
        grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
        grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

        retL, cornersL = cv2.findChessboardCorners(grayL, CHESSBOARD_SIZE, None)
        retR, cornersR = cv2.findChessboardCorners(grayR, CHESSBOARD_SIZE, None)

        if retL and retR:
            objpoints.append(objp)
            cornersL = cv2.cornerSubPix(grayL, cornersL, (11, 11), (-1, -1),
                                        (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001))
            cornersR = cv2.cornerSubPix(grayR, cornersR, (11, 11), (-1, -1),
                                        (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001))
            imgpoints_left.append(cornersL)
            imgpoints_right.append(cornersR)

    print(f"Nalezeno {len(objpoints)} použitelných párů")

    if len(objpoints) < 5:
        print("❌ Nedostatek použitelných snímků pro kalibraci.")
        return

    # Monokalibrace
    retL, mtxL, distL, _, _ = cv2.calibrateCamera(objpoints, imgpoints_left, grayL.shape[::-1], None, None)
    retR, mtxR, distR, _, _ = cv2.calibrateCamera(objpoints, imgpoints_right, grayR.shape[::-1], None, None)

    # Stereo kalibrace
    flags = cv2.CALIB_FIX_INTRINSIC
    retval, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
        objpoints, imgpoints_left, imgpoints_right,
        mtxL, distL, mtxR, distR, grayL.shape[::-1],
        criteria=(cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 100, 1e-5),
        flags=flags
    )

    print("🎯 Kalibrace hotova:")
    print(f"▶ R (rotace):\n{R}")
    print(f"▶ T (translace):\n{T}")

    np.savez("stereo_calibration.npz", mtxL=mtxL, distL=distL, mtxR=mtxR, distR=distR, R=R, T=T, E=E, F=F)

if __name__ == "__main__":
    left_imgs = load_images(LEFT_IMAGES_DIR)
    right_imgs = load_images(RIGHT_IMAGES_DIR)
    calibrate_stereo(left_imgs, right_imgs)
