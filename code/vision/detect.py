"""Module for basic shape detectors."""

import cv2 as cv
import numpy as np
import os

from camera.transforms import BALL_RADIUS_PX

#####HSV Colour Ranges#################
# Blue Player Range
BLUE_LOW_MASK = (80,100,110)
BLUE_HIGH_MASK = (120, 255, 255)

# White Player Range (update this range or change the color)
WHITE_LOW_MASK = (0, 0, 227)
WHITE_HIGH_MASK = (100, 255, 255)

# Red Ball Range (most important one!)
RED_LOW_MASK = (155, 145, 0)
RED_HIGH_MASK = (179, 255, 255)
########################################

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

def detect_rectangles(hsv: np.array) -> list | None:
    """uses hsv color space masking and basic image processing to find
    rectangles in the image
    
    Args:
        hsv (np.array): image in hsv-space for color detection

    Returns:
        list:   list of [x, y, w, h] coords of any squares detected. (x,y) is top
                left corner
    """
    rectangles = []

    kernel = cv.getStructuringElement(cv.MORPH_RECT,(5, 5))
    mask = cv.inRange(hsv, BLUE_LOW_MASK, BLUE_HIGH_MASK)
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel, iterations=2)
    mask = cv.dilate(mask, kernel, iterations=1) # final dilate to connect blocks

    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        rectangles.append(cv.boundingRect(cnt)) # x,y,w,h

    return rectangles


if __name__ == "__main__":
    ######################### testing ##########################
    img_w, img_h = 1280, 720
    cam_index = 2
    if os.name == "nt":
        # for windows to be able to open the camera in a reasonable amount of time
        vc = cv.VideoCapture(cam_index, cv.CAP_DSHOW)
    else:
        vc = cv.VideoCapture(cam_index)
    vc.set(cv.CAP_PROP_FRAME_WIDTH, img_w)
    vc.set(cv.CAP_PROP_FRAME_HEIGHT, img_h)

    flag = False
    if vc.isOpened(): # try to get the first frame
        flag, _ = vc.read()
    
    if flag:
        window_name = "Testing"
        cv.namedWindow(window_name, cv.WINDOW_NORMAL)
        scale_factor = 2

        while True:
            # read in next frame
            rval, frame = vc.read()
            if not rval:
                break

            resized_res = (img_w // scale_factor, img_h // scale_factor)
            resized_frame = cv.resize(frame, resized_res, interpolation=cv.INTER_AREA)
            blurred = cv.edgePreservingFilter(resized_frame, cv.RECURS_FILTER, 40, 0.4)
            hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

            circles = detect_circles(hsv)
            rectangles = detect_rectangles(hsv)
            print(f"circles: {circles}")
            print(f"rectangles: {rectangles}")

            if circles is not None:
                for circ in circles:
                    x, y, r = circ
                    cv.circle(resized_frame, (int(x), int(y)), int(r), (255, 0, 0), 2)

            if rectangles is not None:
                for rect in rectangles:
                    x, y, w, h = rect
                    cv.rectangle(resized_frame, (int(x), int(y)), (int(x + w), int(y + h)), (0, 255, 0), 2)
    
            # shows the original image with the detected objects drawn
            cv.imshow(window_name, resized_frame)
            cv.resizeWindow(window_name, img_w, img_h)

            # check if q key is pressed
            key = cv.waitKey(10)
            if key == ord('q'):
                break

    vc.release()
    cv.destroyAllWindows()
