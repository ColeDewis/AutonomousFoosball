"""This modules stores all the camera related information, like pixel scale
values, transform matrices, and a function to transform from pixel coords to
world coords

TODO: re-measure scales and distances once the arena is fully built. Clean up
this code too

TODO: add info like pixel positions of goals as well
"""

import numpy as np
from pathlib import Path

# ball is often ellipsified due to perspective, and measuring its major and minor
# axes at two opposite diagonal corners of the arena I got like 58px to 72 px
# so I'm just averaging the two for the radius checks in ball detection
BALL_RADIUS_PX = 65 / 2 # divide by two to get length in scaled image

# Pixel to centimeter conversion
X_PX2CM = 36.1 / 416
Y_PX2CM = 28.5 / 337
AVG_PX2CM = (X_PX2CM + Y_PX2CM) / 2

# Camera translation from world origin (in cm's)
CX = 26.9
CY = 42
CZ = 62.9

# Camera Rotation from world origin (rotations can happen in any order, they are
# all with respect to fixed world origin) NOTE: check this
C_RX = np.pi
C_RY = 5 * (np.pi / 180)
C_RZ = -90 * (np.pi / 180)

CAM_MTX = np.loadtxt(str(Path(__file__).parent) + "/calibrate/camera_mat.txt") # make sure this load works
CAM_MTX_INVERSE = np.linalg.inv(CAM_MTX)
DIST_COEF = np.loadtxt(str(Path(__file__).parent) + "/calibrate/distortion_coef.txt")

T = np.array(
    [
        [1, 0, 0, CX],
        [0, 1, 0, CY],
        [0, 0, 1, CZ],
        [0, 0, 0, 1]
    ]
)
RZ = np.array(
    [
        [np.cos(C_RZ), -np.sin(C_RZ), 0, 0],
        [np.sin(C_RZ), np.cos(C_RZ), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ]
)
RX = np.array(
    [
        [1, 0, 0, 0],
        [0, np.cos(C_RX), -np.sin(C_RX), 0],
        [0, np.sin(C_RX), np.cos(C_RX), 0],
        [0, 0, 0, 1]
    ]
)
RY = np.array(
    [
        [np.cos(C_RY), 0, np.sin(C_RY), 0],
        [0, 1, 0, 0],
        [-np.sin(C_RY), 0, np.cos(C_RY), 0],
        [0, 0, 0, 1]
    ]
)
R = T @ RZ @ RY @ RX
R_INVERSE = np.linalg.inv(R)
# this last one is the one that takes us from scaled image coords to camera coords
PX2CAM = np.array([[X_PX2CM, 0, 0, -6.720675540436016036e+02 * X_PX2CM], [0, Y_PX2CM, 0, -3.655077871747401446e+02 * Y_PX2CM], [0, 0, 1, 0], [0, 0, 0, 1]])
CAM2WORLD = T @ RZ @ RX @ RY @ PX2CAM # Correct one somehow? TODO: check this out

def img_to_world(u: int, v: int, z: float = 5.1) -> list:
    """finds the world coordinate of a uv-coord from the image frame

    TODO: update this to have proper transformations, I don't think I have this
    fully right
    
    Args:
        u (int): u-coord (x) of pixel
        v (int): v-coord (y) of pixel
        z (float): the z-coord of the object we are looking at in the real world
                    (since our ball is 5.1cm I made that default)

    Returns:
        list: [x, y] in world frame (since we don't need z-coord)
    """

    world_coords = CAM2WORLD @ np.transpose([u, v, 0, 1])
    # cam_coords = (CAM_MTX_INVERSE @ np.transpose([u, v, 1])) / AVG_PX2CM
    # print(cam_coords)
    # world_coords = R_INVERSE @ np.append(cam_coords, 1)
    # coords = R_INVERSE @ CAM_MTX_INVERSE @ np.transpose([u, v, z])
    # print(coords)
    return list(np.squeeze(world_coords)[:2])

if __name__ == "__main__":
    # testing
    pt = [0, 0, 5.1]
    print(img_to_world(*pt))
