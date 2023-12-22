import numpy as np
import cv2 as cv
from pathlib import Path
from matplotlib import pyplot as plt
from sklearn import linear_model
import glob
import re

from src.vision.detect import detect_circles, find_optimal_circle
from src.vision.camera.transforms import (
    INTRINSIC,
    INTRINSIC_INV,
    DIST_COEF,
)

###############################################################################
######################### Helper Functionality ###############################
###############################################################################

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

# the balls were propped up in each of the images by a fix amount due to container
# so z is a little larger than actual
true_world_coords = np.array([
        [43.4, 5.8, 6.2],
        [43.4, 38.7, 6.2],
        [43.5, 77.0, 6.2],
        [26.3, 77.1, 6.2],
        [5.7, 76.9, 6.2],
        [5.8, 41.3, 6.2],
        [5.8, 6.7, 6.2],
        [25.1, 5.9, 6.2],
        [18.4, 26.0, 6.2],
        [19.5, 58.9, 6.2],
        [35.5, 63.5, 6.2],
        [35.2, 22.2, 6.2],
        [26.4, 42.8, 6.2]
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
    blurred = cv.edgePreservingFilter(frame, cv.RECURS_FILTER, 40, 0.4)
    hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

    circles = detect_circles(hsv)
    if circles is not None:
        x, y, r = find_optimal_circle(circles, img_scale=1)
        img_coords.append(np.array([x, y]))
        cv.circle(frame, (int(x), int(y)), int(r), (255, 0, 0), 2)
        cv.imshow("circle", frame)
        cv.waitKey(0)

#############################################################################
################### Solve for extrinsic matrix ##############################
#############################################################################

print("solve PnP")
ret, rvec1, tvec1 = cv.solvePnP(true_world_coords, np.array(img_coords), INTRINSIC, DIST_COEF)
R_mtx, jac = cv.Rodrigues(rvec1)
Rt = np.column_stack((R_mtx,tvec1))
P_mtx = INTRINSIC.dot(Rt)
np.savetxt("ball_detection_extrinsic.txt", Rt)

###############################################################################
################### Find s values and compute errors ##########################
###############################################################################

s_arr = []
for_error = np.transpose(np.array([0., 0.]))
inv_error = np.transpose(np.array([0., 0.]))
for i, img_coord in enumerate(img_coords):
    # forwards process
    suv = P_mtx @ np.append(true_world_coords[i], 1).T
    s_new = suv[2]
    s_arr.append(s_new)
    uv = suv / s_new
    for_error += np.abs(img_coord - uv[:2])

    # backwards process
    suv_new = np.append(img_coord, 1) * s_new
    cam_coords = np.expand_dims(INTRINSIC_INV @ suv_new, 1)
    cam_coords -= tvec1
    world_est = np.squeeze(np.linalg.inv(R_mtx) @ cam_coords)
    inv_error += np.abs(true_world_coords[i][:2] - world_est[:2])

print(f"Average Forward Error (Computed Extrinsic): {for_error / true_world_coords.shape[0]}")
print(f"Average Inverse Error (Computed Extrinsic): {inv_error / true_world_coords.shape[0]}")

###############################################################################
################### Find plane equation for s values ##########################
###############################################################################

np_img_coords = np.array(img_coords)
x = np_img_coords[:, 0].flatten()
y = np_img_coords[:, 1].flatten()
z = np.array(s_arr).flatten()

X_data = np.column_stack((x, y))
Y_data = z

reg = linear_model.LinearRegression().fit(X_data, Y_data)

a, b = reg.coef_
c = reg.intercept_
print(f"Coefficients of the plane: {a, b}")
print(f"Intercept: {c}")

###############################################################################
################### Plot the data and the fit plane ##########################
###############################################################################

# Create a 3D scatter plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x, y, z, c='r', marker='o', label='Data Points')

# Create a meshgrid for the plane
x_plane = np.linspace(min(x), max(x), 100)
y_plane = np.linspace(min(y), max(y), 100)
x_plane, y_plane = np.meshgrid(x_plane, y_plane)
z_plane = a * x_plane + b * y_plane + c
ax.plot_surface(x_plane, y_plane, z_plane, alpha=0.5, color='b', label='Plane')

# Set labels for each axis
ax.set_xlabel('U Coordinates (pixels)')
ax.set_ylabel('V Coordinates (pixels)')
ax.set_zlabel('Z Coordinates (cm)')

# Set plot title
ax.set_title('Z Camera Coordinates of Points in Playing Area')

# Add a legend
ax.legend()
plt.show()
