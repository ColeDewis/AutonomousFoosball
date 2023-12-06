import numpy as np
import cv2 as cv
from pathlib import Path
import glob
import re

from transforms import (
    CX,
    CY,
    CZ,
    BALL_RADIUS_PX,
    BALL_DIAMTER_CM,
    INTRINSIC,
    INTRINSIC_INV,
    EXTRINSIC_MTX,
    DIST_COEF,
    CAM_POSE_MTX,
    world_to_image,
    image_to_world
)

RED_LOW_MASK = (155, 145, 0)
RED_HIGH_MASK = (179, 255, 255)

###############################################################################
######################### Helper Functionality ###############################
###############################################################################

def detect_circles(hsv: np.array) -> list | None:
    """uses hsv color space masking and basic image processing to find
    circles in the image
    
    Args:
        hsv (np.array): blurred image in hsv-color space for color detection

    Returns:
        None: no ball detected
        list: list of [x, y, radius] coord of any detected circles
    """
    # mask based on red color and then use morph operations to clean mask
    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE,(7,5))
    mask = cv.inRange(hsv, RED_LOW_MASK, RED_HIGH_MASK)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel, iterations=3)

    circles = cv.HoughCircles(mask, cv.HOUGH_GRADIENT, 1.5, 300, param1=100, param2=20, minRadius=10, maxRadius=50)
    if circles is not None:
        circles = np.squeeze(circles)
        if circles.ndim == 1:
            circles = [list(circles)]

    return circles

def find_optimal_circle(circles: np.array, prev_pos: np.array = None, img_scale: int = 2) -> list:
    """finds the closest matched circle in case multiple are detected
    
    Args:
        circles (np.array): array of all detected circles
        prev_pos (np.array): [x, y] coord of previous ball detection
    
    Returns:
        list: [x, y] coords of best matched circle
    """
    min_dist = np.inf
    for circ in circles: # circ = [x, y, rad]
        if prev_pos is None:
            # no detection to match so far, pick one with closest radius
            diff = abs(circ[2] - BALL_RADIUS_PX / img_scale)
            if diff < min_dist:
                min_dist = diff
                best = circ
        else:
            # we have previous detections, find best match according to
            # distance from previous point (since likely very close still)
            dist = np.sum(np.square(circ[:2] - prev_pos))
            if dist < min_dist:
                min_dist = dist
                best = circ

    return list(best)

def tryint(s):
    try:
        return int(s)
    except:
        return s

def alphanum_key(s):
    """ Turn a string into a list of string and number chunks.
        "z23a" -> ["z", 23, "a"]
    """
    return [ tryint(c) for c in re.split('([0-9]+)', s) ]

def sort_nicely(l):
    """ Sort the given list in the way that humans expect.
    """
    l.sort(key=alphanum_key)
    return l

###############################################################################
########################## World Coordinate Preparation #######################
###############################################################################

# # z value is euclidean distance to camera center, not z value in world coords
# measured_coords = np.array([
#         [43.4, 5.8, 68.0],
#         [43.4, 38.7, 59.0],
#         [43.5, 77.0, 69.1],
#         [26.3, 77.1, 67.3],
#         [5.7, 76.9, 72.5],
#         [5.8, 41.3, 62.0],
#         [5.8, 6.7, 71.7],
#         [25.1, 5.9, 67.8],
#         [18.4, 26.0, 61.1],
#         [19.5, 58.9, 60.6],
#         [35.5, 63.5, 62.2],
#         [35.2, 22.2, 61.5],
#         [26.4, 42.8, 57.2]
#     ]
# )

# the balls were propped up in each of the images by a fix amount due to container
true_world_coords = np.array([
        [43.4, 5.8, 5.5],
        [43.4, 38.7, 5.5],
        [43.5, 77.0, 5.5],
        [26.3, 77.1, 5.5],
        [5.7, 76.9, 5.5],
        [5.8, 41.3, 5.5],
        [5.8, 6.7, 5.5],
        [25.1, 5.9, 5.5],
        [18.4, 26.0, 5.5],
        [19.5, 58.9, 5.5],
        [35.5, 63.5, 5.5],
        [35.2, 22.2, 5.5],
        [26.4, 42.8, 5.5]
    ]
)

###############################################################################
################### Detecting Image Coordinates ##############################
###############################################################################

# detect circles and get image coordinates
img_path = str(Path(__file__).parent) + "/images/"
images = sort_nicely(list(glob.glob(str(img_path) + '/*.png')))
img_coords = []
for i, fname in enumerate(images):
    frame = cv.imread(fname)
    # blurred = cv.GaussianBlur(frame, (5, 5), 0)
    blurred = cv.edgePreservingFilter(frame, cv.RECURS_FILTER, 40, 0.4)
    hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

    circles = detect_circles(hsv)
    if circles is not None:
        x, y, r = find_optimal_circle(circles, img_scale=1)
        img_coords.append(np.array([x, y]))
        # cv.circle(frame, (int(x), int(y)), int(r), (255, 0, 0), 2)
        # cv.imshow("circle", frame)
        # cv.waitKey(0)


print("solvePNP")
ret, rvec1, tvec1 = cv.solvePnP(true_world_coords, np.array(img_coords), INTRINSIC, DIST_COEF)
R_mtx, jac = cv.Rodrigues(rvec1)
Rt = np.column_stack((R_mtx,tvec1))
P_mtx = INTRINSIC.dot(Rt)

print(f"Measured Extrinsic: {EXTRINSIC_MTX}")
print(f"Computed Extrinsic: {Rt}")

###############################################################################
######################## Finding average s value ##############################
###############################################################################

print("\n################# Finding optimal s value ####################\n")
s = 0
img_to_world_err = np.array([0., 0.])
for i, img_coord in enumerate(img_coords):
    print("\n---------- True Values ---------------")
    print(f"True Image Coordinate: {img_coord}")
    print(f"True World Coordinate: {true_world_coords[i]}")

    suv = P_mtx @ np.append(true_world_coords[i], 1).T
    s_new = suv[2]
    uv = suv / s_new
    print(f"Image Estimate: {uv}")
    print(f"Scale: {s_new}")
    s += s_new / len(img_coords)

    suv_new = np.append(img_coord, 1) * s_new
    cam_coords = np.expand_dims(INTRINSIC_INV @ suv_new, 1)
    cam_coords -= tvec1
    world_coords = np.squeeze(np.linalg.inv(R_mtx) @ cam_coords)
    print(f"World Estimate: {np.squeeze(world_coords)}")

    # computing error
    diff = np.abs(world_coords[:2] - true_world_coords[i][:2])
    print(f"DIFFERENCE: {diff}")
    img_to_world_err += diff / len(img_coords)

print(f"Average error: {img_to_world_err}")
print(f"Average scale value: {s}")
