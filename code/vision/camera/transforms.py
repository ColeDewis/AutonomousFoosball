"""This modules stores all the camera related information, like pixel scale
values, transform matrices, and a function to transform from pixel coords to
world coords
"""

import numpy as np
from pathlib import Path

# ball is often ellipsified due to perspective, and measuring its major and minor
# axes at two opposite diagonal corners of the arena I got like 58px to 72 px
# so I'm just averaging the two for the radius checks in ball detection
BALL_RADIUS_PX = 65 / 2 # divide by 2 since I measured diameter in image
BALL_DIAMTER_CM = 5.1

# Pixel to centimeter conversion
X_PX2CM = 36.1 / 416
Y_PX2CM = 28.5 / 337
AVG_PX2CM = (X_PX2CM + Y_PX2CM) / 2

# Plane Equation coefficients
A = -0.0006659935
B = 0.010184872
C = 58.39907455444336 + 1.3 # extra +1.3 since the balls in the image were a bit above normal

INTRINSIC = np.loadtxt(str(Path(__file__).parent) + "/camera_mat.txt")
INTRINSIC_INV = np.linalg.inv(INTRINSIC)
DIST_COEF = np.loadtxt(str(Path(__file__).parent) + "/distortion_coef.txt")

EXTRINSIC = np.loadtxt(str(Path(__file__).parent) + "/ball_detection_extrinsic.txt")
EXTRINSIC = np.row_stack((EXTRINSIC, [0, 0, 0, 1]))
CAM_POSE_MTX = np.linalg.inv(EXTRINSIC)


def world_to_image(x: float, y: float, z: float = 5.1) -> list:
    """gets image coordinates from world coordinates
    Args:
        u (float): x coordinate in world frame
        v (float): y coordinate in world frame
        z (float): z coordinate in world frame

    Returns:
        list: [u, v] coords in image frame
    """
    cam_coords = EXTRINSIC @ np.array([x, y, z, 1]).T
    scaled_img_coords = INTRINSIC @ cam_coords[:3] # drop homogeneous coord
    img_coords = (scaled_img_coords / scaled_img_coords[2]).astype(int)
    return list(img_coords[:2])


def image_to_world(u: int, v: int, z: float = 5.1) -> list:
    """gets world coordinates from image coordinates

    NOTE: Since our camera was slightly rotated, we best fit a plane to some
    measured scale values to get better estimates
    
    Args:
        u (int): x coordinate in image
        v (int): y coordinate in image
        z (float): z value (in world frame) of object this uv-coord corresponds to

    Returns:
        list: [x, y] coords in world frame
    """
    s = A * u + B * v + C
    cam_coords = INTRINSIC_INV @ (s * np.array([u, v, 1]).T)
    world_coords = CAM_POSE_MTX @ np.append(cam_coords, 1)
    return list(world_coords[:2])


if __name__ == "__main__":
    ######################### TESTING ######################################
    import cv2 as cv
    from pathlib import Path
    from matplotlib import pyplot as plt

    # this version is specific for this testing I did
    def detect_rectangles(mask: np.array) -> list | None:
        rectangles = []

        kernel = cv.getStructuringElement(cv.MORPH_RECT, (5, 5))
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel, iterations=1)

        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            rect = cv.boundingRect(cnt)

            if rect[2] * rect[3] < 300:
                continue

            if rect[0] < mask.shape[1] / 2 - 150 or rect[0] > mask.shape[1] / 2 + 200:
                continue

            if rect[1] < mask.shape[0] / 2 - 150 or rect[1] > mask.shape[0] / 2 + 100:
                continue

            rectangles.append(rect) # x,y,w,h

        return rectangles

    true_coords = np.array(
        [
            [16, 52.3, 0],
            [16.5, 31.2, 0],
            [20.9, 45.7, 0],
            [21.4, 37, 0],
            [28.5, 45.7, 0],
            [29, 37, 0],
            [32.2, 52.4, 0],
            [33.2, 31.2, 0]
        ]
    )

    ############################################################################
    ###################### Find Image Coordinates ##############################
    ############################################################################

    dir_path = str(Path(__file__).parent)
    img = cv.imread(dir_path + "/test.png")
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    _, thresh = cv.threshold(gray, 100, 255, cv.THRESH_BINARY_INV)

    image_coords = []
    rectangles = detect_rectangles(thresh)
    for i, (x, y, w, h) in enumerate(rectangles):
        c_x = x + w // 2
        c_y = y + h // 2
        image_coords.append(np.array([c_x, c_y]))
        print(f"Image Coords of {i}: ({c_x}, {c_y})")

        cv.rectangle(img, (int(x), int(y)), (int(x+w), int(y+h)), (255, 0, 0), 2)
        cv.circle(img, (c_x, c_y), 2, (0, 0, 255), -1)
        cv.putText(img, f"{i}", (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

    # this just draws the center of projection on the image
    cv.circle(img, (int(INTRINSIC[0, 2]), int(INTRINSIC[1, 2])), 2, (0, 0, 255), -1)
    cv.imshow("Rectangles", img)
    cv.waitKey(0)

    ############################################################################
    ###################### Get Extrinsic Error ########################
    ############################################################################

    for_error = np.transpose(np.array([0., 0.]))
    inv_error = np.transpose(np.array([0., 0.]))
    for i, img_coord in enumerate(image_coords):
        img_est = world_to_image(*np.squeeze(true_coords[i]))
        for_error += np.abs(img_coord - np.array(img_est))
        world_est = image_to_world(*img_coord, true_coords[i][2])
        inv_error += np.abs(true_coords[i][:2] - np.array(world_est))

    print(f"Average Forward Error: {for_error / true_coords.shape[0]}")
    print(f"Average Inverse Error: {inv_error / true_coords.shape[0]}")

    ############################################################################
    ################# Compute Extrinsic from square images #####################
    ############################################################################

    ret, rvec1, tvec1 = cv.solvePnP(true_coords, np.array(image_coords).astype(float), INTRINSIC, DIST_COEF)
    R_mtx, _ = cv.Rodrigues(rvec1)
    Rt = np.column_stack((R_mtx,tvec1))
    np.savetxt("sq_detection_extrinsic.txt", Rt)
    P_mtx = INTRINSIC @ Rt

    ############################################################################
    ###################### Get Computed Extrinsic Error ########################
    ############################################################################

    s_arr = []
    for_error = np.transpose(np.array([0., 0.]))
    inv_error = np.transpose(np.array([0., 0.]))
    for i, img_coord in enumerate(image_coords):
        suv = P_mtx @ np.append(true_coords[i], 1).T
        s_new = suv[2]
        s_arr.append(s_new)
        uv = suv / s_new
        for_error += np.abs(img_coord - uv[:2])

        suv_new = np.append(img_coord, 1) * s_new
        cam_coords = np.expand_dims(INTRINSIC_INV @ suv_new, 1)
        cam_coords -= tvec1
        world_est = np.squeeze(np.linalg.inv(R_mtx) @ cam_coords)
        inv_error += np.abs(true_coords[i][:2] - world_est[:2])

    print(f"Average Forward Error (Computed Extrinsic): {for_error / true_coords.shape[0]}")
    print(f"Average Inverse Error (Computed Extrinsic): {inv_error / true_coords.shape[0]}")

    ############################################################################
    ###################### Plot Scale Values ###################################
    ############################################################################

    np_img_coords = np.array(image_coords)
    ax = plt.figure().add_subplot(projection='3d')
    ax.scatter(np_img_coords[:, 0], np_img_coords[:, 1], s_arr, color = "blue")
    plt.xlabel("U Coordinate (pixels)")
    plt.ylabel("V Coordinate (pixels)")
    plt.title("Z Camera Coordinates of Points Around Arena")
    plt.show()
