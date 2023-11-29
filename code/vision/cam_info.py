"""This modules stores all the camera related information, like pixel scale
values, transform matrices, and a function to transform from pixel coords to
world coords

TODO: re-measure scales and distances once the arena is fully built. Clean up
this code too

TODO: add info like pixel positions of goals as well
"""

import numpy as np

# ball is often ellipsified due to perspective, and measuring its major and minor
# axes at two opposite diagonal corners of the arena I got like 58px to 72 px
# so I'm just averaging the two for the radius checks in ball detection
BALL_RADIUS_PX = 65 / 2 # divide by two to get length in scaled image
PLAYER_SIDE_LEN_PX = 60 / 2 # ditto (NOTE: change when we lower carriage)

# Pixel to centimeter conversion
X_PX2CM = 36.1 / 416
Y_PX2CM = 28.5 / 337

# Camera translation from world origin (in cm's)
CX = 26.9
CY = 42
CZ = 62.9

# TODO: make sure rotation matrix for y is correct
# NOTE: could throw all this camera info into a class and have it implement
# the pixel to world coord transform
# defining transforms (clean this up later)
T1 = np.array([[1, 0, 0, CX], [0, 1, 0, CY], [0, 0, 1, CZ], [0, 0, 0, 1]])
R1 = np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
R2 = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
R3 = np.array([[np.cos(np.pi / 36), 0, np.sin(np.pi / 36), 0], [0, 1, 0, 0], [-np.sin(np.pi / 36), 0, np.cos(np.pi / 36), 0], [0, 0, 0, 1]])
# this last one is the one that takes us from scaled image coords to camera coords
PX2CAM = np.array([[X_PX2CM, 0, 0, -6.720675540436016036e+02 * X_PX2CM], [0, Y_PX2CM, 0, -3.655077871747401446e+02 * Y_PX2CM], [0, 0, 1, 0], [0, 0, 0, 1]])
CAM2WORLD = T1 @ R1 @ R2 @ R3 @ PX2CAM

def img_to_world(u: int, v: int, is_vec: bool = False) -> list:
    """finds the world coordinate of a uv-coord from the image frame

    TODO: update this so we can handle players being closer to camera (i.e. add
    a z value instead of 0 in the transpose maybe?)

    TODO: update this to have proper transformations, I don't think I have this
    fully right
    
    Args:
        u (int): u-coord (x) of pixel
        v (int): v-coord (y) of pixel

    Returns:
        list: [x, y] in world frame (since we don't need z-coord)
    """
    if is_vec:
        coords = CAM2WORLD @ np.transpose([u, v, 0, 0])
    else:
        coords = CAM2WORLD @ np.transpose([u, v, 0, 1])
    return list(np.squeeze(coords)[:2])
