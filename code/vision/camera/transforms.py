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

# Camera translation from world origin (in cm's)
CX = 22.2
# CX = 24 # potentially adding some offset still may help
CY = 42
# CY = 41 # ditto
CZ = 62.9
# CZ = 62.4 # I'm not sure which of these two is more accurate

# Camera Rotations from world origin
C_RX = np.pi
C_RY = 5 * (np.pi / 180)
C_RZ = -90 * (np.pi / 180)

INTRINSIC = np.loadtxt(str(Path(__file__).parent) + "/calibrate/camera_mat.txt")
INTRINSIC_INV = np.linalg.inv(INTRINSIC)
DIST_COEF = np.loadtxt(str(Path(__file__).parent) + "/calibrate/distortion_coef.txt")

# NOTE: I measured the Camera Pose in World coordinates, meaning that the extrinsic
# matrix made from this will correspond to the one required to go from camera coords
# to world coords (the inverse operation). To get how the world points should
# transform to camera coordinates, we can just invert the homogeneous matrix
# NOTE: For whatever reason I'm getting much less error when assuming the camera
# is parallel to the table even though there is a slight angle in reality
# CAM_POSE_MTX = np.array(
#     [
#         [0, -np.cos(C_RY), -np.sin(C_RY), CX],
#         [-1, 0, 0, CY],
#         [0, np.sin(C_RY), -np.cos(C_RY), CZ],
#         [0, 0, 0, 1]
#     ]
# )
CAM_POSE_MTX = np.array(
    [
        [0, -1, 0, CX],
        [-1, 0, 0, CY],
        [0, 0, -1, CZ],
        [0, 0, 0, 1]
    ]
)
EXTRINSIC_MTX = np.linalg.inv(CAM_POSE_MTX)


def world_to_image(x: float, y: float, z: float = 5.1) -> list:
    """gets image coordinates from world coordinates
    Args:
        u (float): x coordinate in world frame
        v (float): y coordinate in world frame
        z (float): z coordinate in world frame

    Returns:
        list: [u, v] coords in image frame
    """
    cam_coords = EXTRINSIC_MTX @ np.array([x, y, z, 1]).T
    scaled_img_coords = INTRINSIC @ cam_coords[:3] # drop homogeneous coord
    img_coords = (scaled_img_coords / scaled_img_coords[2]).astype(int)
    return list(img_coords[:2])


def image_to_world(u: int, v: int, z: float = 5.1) -> list:
    """gets world coordinates from image coordinates

    NOTE: currently assuming uniform object distance from camera, so they scale
    value is fixed once z is known. If this proves to do poorly at image edges,
    I could take measurements all over the arena and fit a function to the results,
    and compute the scale from that
    
    Args:
        u (int): x coordinate in image
        v (int): y coordinate in image
        z (float): z value (in world frame) of object this uv-coord corresponds to

    Returns:
        list: [x, y] coords in world frame
    """
    s = CZ - z + 1 # adding 1 from empirical testing (gives lower error)
    cam_coords = INTRINSIC_INV @ (s * np.array([u, v, 1]).T)
    world_coords = CAM_POSE_MTX @ np.append(cam_coords, 1)
    return list(world_coords[:2])

if __name__ == "__main__":
    ######################### TESTING ######################################
    import cv2 as cv
    from pathlib import Path

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

    # points I measured on sheet of paper in our arena (only care about x and y, but
    # I may be wrong about that and I may need the hypotenuses for getting proper
    # extrinsic parameters since my calibration results are kind of scuffed)
    true_coords = {
        0: np.array([16, 52.3, 0]),
        1: np.array([16.5, 31.2, 0]),
        2: np.array([20.9, 45.7, 0]),
        3: np.array([21.4, 37, 0]),
        4: np.array([28.5, 45.7, 0]),
        5: np.array([29, 37, 0]),
        6: np.array([32.2, 52.4, 0]),
        7: np.array([33.2, 31.2, 0]),
    }

    dir_path = str(Path(__file__).parent)
    img = cv.imread(dir_path + "/test.png")
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    _, thresh = cv.threshold(gray, 100, 255, cv.THRESH_BINARY_INV)

    # find squares
    rectangles = detect_rectangles(thresh)

    # do some testing
    image_coords = []
    error = np.transpose(np.array([0., 0.]))
    for i, (x, y, w, h) in enumerate(rectangles):
        c_x = x + w // 2
        c_y = y + h // 2
        image_coords.append(np.array([c_x, c_y]))
        print(f"Image Coords of {i}: ({c_x}, {c_y})")

        cv.rectangle(img, (int(x), int(y)), (int(x+w), int(y+h)), (255, 0, 0), 2)
        cv.circle(img, (c_x, c_y), 2, (0, 0, 255), -1)
        cv.putText(img, f"{i}", (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)

        world_est = image_to_world(c_x, c_y, true_coords[i][2]) # z was zero for these points
        print(f"World Coord estimate of {i}: {world_est}")
        print(f"True World Coord: {true_coords[i]}\n")

        error += np.abs(true_coords[i][:2] - np.array(world_est))

    print(f"Average Error: {error / len(true_coords)}")

    # this just draws the center of projection on the image
    cv.circle(img, (int(INTRINSIC[0, 2]), int(INTRINSIC[1, 2])), 2, (0, 0, 255), -1)
    cv.imshow("Rectangles", img)
    cv.waitKey(0)

    ############### finding optimal z-val For normalizing uv-coords ###########
    # NOTE: this section is useful if I actually measure the distance from points
    # to the camera center (z-direction) and use trig to compute the z values
    # of the points, instead of just assuming uniform z like I have
    ###########################################################################
    print("\n################# Finding optimal s value ####################\n")
    s = 0
    for i, pt in true_coords.items():
        print("\n---------- True Values ---------------")
        print(f"True Image Coordinate: {image_coords[i]}")
        print(f"True World Coordinate: {pt}")

        # forwards from world to image
        print(f"\nComputing forwards for point {i}")
        print(f"Image Estimate: {world_to_image(*list(pt))}")
        cam_coords = EXTRINSIC_MTX @ np.array([x, y, 0, 1]).T
        scaled_uv = INTRINSIC @ cam_coords[:3] # drop homogeneous coord
        print(f"Scale value s: {scaled_uv[2]}")
        s += scaled_uv[2] / len(true_coords) # averaging scale values for final estimate

        # backwards from image to world
        print(f"\nComputing backwards for point {i}")
        print(f"World Estimate: {image_to_world(*list(image_coords[i])), 0}")

    
    print(f"Average scale: {s}")
