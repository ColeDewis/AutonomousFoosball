import numpy as np
import cv2 as cv
import glob

from pathlib import Path

file_path = Path(__file__)
img_path = file_path.parent / "images/"

# may have printed off a non-square chessboard
SQR_LEN = 2.7 # cm
SQR_HEIGHT = 2.5 # cm
CB_SIZE = (9, 7) # how many corners to look for (where black square corners meet)

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
# except multiply by chessboard square size for proper units
objp = np.zeros((CB_SIZE[0] * CB_SIZE[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:CB_SIZE[0],0:CB_SIZE[1]].T.reshape(-1,2)
objp[:, 0] *= SQR_LEN
objp[:, 1] *= SQR_HEIGHT

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob(str(img_path) + '/*.png')

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, CB_SIZE, None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv.drawChessboardCorners(img, CB_SIZE, corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
print(f"\nCamera Matrix: {mtx}")
print(f"\nDistortion Coefficients: {dist}")

np.savetxt("camera_mat.txt", mtx)
np.savetxt("distortion_coef.txt", dist)

mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error
print( "total error: {}".format(mean_error/len(objpoints)))
