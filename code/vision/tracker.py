"""
Ideas:
- potentially try and base position estimations based on current point and
trajectory estimation by timestepping trajectory and weighing the current point
with the trajectory estimate
    - for players (and potentially ball), when their velocity is 0 (or super low),
    I should estimate position over time by just averaging point detections (like
    what we did for lab 3). Could weigh points based on how far they are from
    the average so far (farther = less weight) although regular average is
    probably fine
- can also try background subtraction + color detection
- could build very simple trajectory models of players where I just fit a line
to the points we query from it and then use that line as a way to filter out noisy
square detections (if any)
"""

import cv2 as cv
import numpy as np
import threading
import time

# scuffed way of importing from same level package since ".." wouldn't work
from pathlib import Path
import sys
_parent_dir = Path(__file__).parent.parent.resolve()
sys.path.insert(0, str(_parent_dir))
from utils.decay_functions import ExponentialDecay
from utils.weighted_moving_average import WeightedMovingAverage
sys.path.remove(str(_parent_dir))

#####HSV Colour Ranges#################
# Blue Player Range
BLUE_LOW_MASK = (80,100,110)
BLUE_HIGH_MASK = (120, 255, 255)

# White Player Range (update this range)
WHITE_LOW_MASK = (0, 0, 227)
WHITE_HIGH_MASK = (100, 255, 255)

# Red Ball Range (most important one!)
RED_LOW_MASK = (155, 145, 0)
RED_HIGH_MASK = (179, 255, 255)
########################################

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

class Tracker:
    """class for handling object detection and tracking functionality
    
    NOTE: the detection functionality was split into two functions to allow for
    different kernel sizes for morph operations and different color masks
    for detecting the ball vs. the players

    NOTE: to save time (and also for ease of programming checks in object
    detection routines) the conversion to world coordinates will happen in
    the self.get_object_location() method, and not when I add points to the
    object location attributes
    """

    ball_id = 0
    player1_id = 1
    player2_id = 2

    def __init__(self, cam_index: int = 0, img_w: int = 1280, img_h: int = 720):
        # TODO: build ball trajectory model class
        # Just using a single point for the ball, the ball trajectory model can
        # handle noisy estimates (i.e. don't add a point if it is too far from
        # what the model expects. Need to consider x,y coords and radius for this.
        # if radius is smaller and x,y is close, this is probably fine. If radius
        # is super big there was probably a fair amount of noise
        self.ball_location = None
        self.ball_trajectory = None

        # for the players just do a WMA that changes depending on how much it's
        # location changes since the previous one (i.e if moving, use exponential,
        # if stopped use constant average with the expontentialDecay.output()
        # value as the first point in the new constant average)
        # TODO: write new decay function that is a mix of two
        self.player1_location = WeightedMovingAverage(ExponentialDecay, 2)
        self.player2_location = WeightedMovingAverage(ExponentialDecay, 2)

        # set up the camera (should be fine to do this out here since the thread
        # only ever reads from the camera object)
        self.img_w = img_w
        self.img_h = img_h
        self.vc = cv.VideoCapture(cam_index)
        self.vc.set(cv.CAP_PROP_FRAME_WIDTH, img_w)
        self.vc.set(cv.CAP_PROP_FRAME_HEIGHT, img_h)

        flag = False
        if self.vc.isOpened(): # try to get the first frame
            flag, _ = self.vc.read()

        if flag:
            self.flag = threading.Event()
            self.thread = threading.Thread(target=self.track_objects, args=(self.flag,))
            self.thread.start()
        else:
            print("[ERROR] Could not open video capture")
            self.vc.release()

    def track_objects(self, flag: threading.Event):
        """handles detecting objects and updating their current positions and
        motion estimates. This should be the main function used on the vision side
        of things.

        It will start the video capture process and detect the ball and the two
        players every frame. It will update the locations of these entities as
        weighted moving averages, and this state will be queryable from the object
        itself. It will also be in charge of updating the objects estimated
        trajectories every frame

        NOTE: we may be able to get away with updates every 5 frames or so, depending
        on how slow things run in practice

        Args:
            flag (threading.Event): a flag for telling us when to stop the video capture process
        """
        # for whatever reason, if I have this as class state python deadlocks somehow
        # this lets us output a constant sized window for the user to see
        window_name = "Gameplay"
        cv.namedWindow(window_name, cv.WINDOW_NORMAL)
        scale_factor = 2

        while not flag.is_set():
            # read in next frame
            rval, frame = self.vc.read()
            if not rval:
                break

            # for saving compute time. Will cause rounding errors when converting
            # coordinates back to original size but whatever
            resized_res = (self.img_w // scale_factor, self.img_h // scale_factor)
            resized_frame = cv.resize(frame, resized_res, interpolation=cv.INTER_AREA)

            # age previous locations and prune old points
            self.player1_location.age_points()
            self.player1_location.prune(age_threshold=10)
            self.player2_location.age_points()
            self.player2_location.prune(age_threshold=10)

            # blurred = cv.GaussianBlur(frame, (5, 5), 0)
            blurred = cv.edgePreservingFilter(resized_frame, cv.RECURS_FILTER, 40, 0.4)
            hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

            self.ball_location = self.detect_ball(resized_frame, hsv)
            if self.ball_location is not None:
                # scale back to original resolution pixels
                self.ball_location = list(np.array(self.ball_location) * scale_factor)
            # TODO: add location update to trajectory model

            player1_location, player2_location = self.detect_players(resized_frame, hsv)
            if player1_location is not None:
                player1_scaled_loc = list(np.array(player1_location) * scale_factor)
                self.player1_location.add_point(player1_scaled_loc)
            if player2_location is not None:
                player2_scaled_loc = list(np.array(player2_location) * scale_factor)
                self.player2_location.add_point(player2_scaled_loc)

            # shows the original image with the detected objects drawn
            cv.imshow(window_name, resized_frame)
            cv.resizeWindow(window_name, self.img_w, self.img_h)

            # check if q key is pressed
            key = cv.waitKey(10)
            if key == ord('q'):
                break

        # cleanup
        self.vc.release()
        cv.destroyAllWindows()

    def stop_tracking(self):
        """sends signal to any threads created by this object that they should
        quit their video capture loops and clean up
        """
        self.flag.set()
        self.thread.join()
        self.vc.release()
        cv.destroyAllWindows()
        print("Tracking Complete")

    def detect_ball(self, frame: np.array, hsv: np.array) -> list | None:
        """uses hsv color space masking and basic image processing to find
        the ball in the image frame
        
        Args:
            frame (np.array): original image frame
            hsv (np.array): image in hsv-color space for color detection

        Returns:
            None: no ball detected
            list: [x, y, radius] coord of center of ball and radius
        """
        # mask based on red color and then use morph operations to clean mask
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE,(7,5))
        mask = cv.inRange(hsv, RED_LOW_MASK, RED_HIGH_MASK)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel, iterations=3)

        # TODO: play with parameters to find best ones
        # NOTE: can probably get rid of a lot of the checks below (just find one
        # closest to trajectory estimate)
        circles = cv.HoughCircles(mask, cv.HOUGH_GRADIENT, 1.5, 300, param1=100, param2=20, minRadius=10, maxRadius=50)
        if circles is None:
            return None # no circles found
        else:
            # find the closest matched circle in case multiple are detected
            min_dist = np.inf
            prev = None
            if self.ball_location is not None:
                prev = np.append(np.array(self.ball_location), BALL_RADIUS_PX)

            circles = np.squeeze(circles)
            if circles.ndim == 1:
                circles = [circles] # stupid bug fix (for when just one circle)

            for circ in circles: # circ = [x, y, rad]
                if prev is None:
                    # no detection to match so far, pick one with closest radius
                    diff = abs(circ[2] - BALL_RADIUS_PX)
                    if diff < min_dist:
                        min_dist = diff
                        best = circ
                else:
                    # we have previous detections, find best match according to
                    # distance from previous point (since likely very close still)
                    # and radius (using euclidean squared distance)
                    dist = np.sum(np.square(circ - prev))
                    if dist < min_dist:
                        min_dist = dist
                        best = circ

        cv.circle(frame, (int(best[0]), int(best[1])), int(best[2]), (255, 0, 0), 2)

        return best[:2] # just want x and y

    def detect_players(self, frame: np.array, hsv: np.array) -> tuple | None:
        """uses hsv color space masking and basic image processing to find
        player1 and player2 in the image frame.
        
        NOTE: player1 is the right player in the image frame, and player2 is the
        left. Both players will be of the same color for simplicity in the code

        TODO: finish square detection
        
        Args:
            frame (np.array): original image frame (for drawing purposes)
            hsv (np.array): image in hsv-space for color detection

        Returns:
            tuple: the xy-coords of both player1 and player2 as
                    ([x1_center, y1_center, w, h], [x2_center, y2_center, w, h])
                    if one player or no players found, their list is replaced
                    by None
        """
        player1_loc, player2_loc = None, None

        kernel = cv.getStructuringElement(cv.MORPH_RECT,(5, 5))
        mask = cv.inRange(hsv, BLUE_LOW_MASK, BLUE_HIGH_MASK)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel, iterations=3)

        cv.imshow("player mask", mask)
        cv.waitKey(10)

        # NOTE: update this once you have simple trajectory model for players
        contours, _ = cv.findContours(mask, 1, 2) # change these to the enum strings
        for cnt in contours:
            x1, y1 = cnt[0][0]
            # NOTE: figure out what this does
            approx = cv.approxPolyDP(cnt, 0.01*cv.arcLength(cnt, True), True)

            if len(approx) == 4:
                x, y, w, h = cv.boundingRect(cnt) # x,y is top left corner
                center_x, center_y = x + w / 2, y + h / 2
                ratio = float(w)/h

                if ratio < 0.8 and ratio > 1.2 or w * h < 100:
                    # not close enough to square, likely noise 
                    # NOTE: update this check to something related to square size
                    # and trajectory model once you have those measurements and
                    # code written
                    continue
                
                frame = cv.drawContours(frame, [cnt], -1, (0,255,255), 2)
                if center_x >= frame.shape[1] / 2:
                    # right player
                    player1_loc = [center_x, center_y]
                    cv.putText(frame, 'Player 1', (x1, y1), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                else:
                    # left player
                    player2_loc = [center_x, center_y]
                    cv.putText(frame, 'Player 2', (x1, y1), cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        return player1_loc, player2_loc
    
    def get_object_location(self, obj_id: int) -> list | None:
        """query most recent location of an object. Object ID options are:
        -   Tracker.ball_id
        -   Tracker.player1_id
        -   Tracker.player2_id

        NOTE: When someone queries this function, they will not want to have to
        deal with handling None cases, so just query trajectory models based
        on time since last detection to get estimated location and return that
        (could perhaps return a confidence value along with the position that
        could be used as weighting for any planning related stuff)

        Args:
            obj_id (int): the ID of the object whose location you want

        Returns:
            None: incorrect object ID or object has no detections yet
            list: [x, y] coordinate of object in world frame
        """
        location = None
        if obj_id == Tracker.ball_id:
            # NOTE: use trajectory model to get estimate if ball location at timestep
            # is None (i.e. we didn't detect anything). Use # of frames since
            # last detection mixed with the trajectory model to get estimate
            if self.ball_location is not None:
                location = self._img_to_world(*self.ball_location)
        elif obj_id == Tracker.player1_id:
            player1_loc = self.player1_location.output()
            if player1_loc is not None:
                location = self._img_to_world(*player1_loc)
        elif obj_id == Tracker.player2_id:
            player2_loc = self.player2_location.output()
            if player2_loc is not None:
                location = self._img_to_world(*player2_loc)
        else:
            print("[WARNING] incorrect object id for location query")

        return location
        
    def _img_to_world(self, u: int, v: int) -> list:
        """finds the world coordinate of a uv-coord from the image frame
        
        Args:
            u (int): u-coord (x) of pixel
            v (int): v-coord (y) of pixel

        Returns:
            list: [x, y] in world frame (since we don't need z-coord)
        """
        coords = CAM2WORLD @ np.transpose([u, v, 0, 1])
        return list(np.squeeze(coords)[:2])

if __name__ == "__main__":
    # for testing
    tracker = Tracker(2)
    print("Tracker set up, object detection commencing ...")
    try:
        while tracker.thread.is_alive():
            print(f"Ball is at: {tracker.get_object_location(Tracker.ball_id)}")
            print(f"player1 is at: {tracker.get_object_location(Tracker.player1_id)}")
            print(f"player2 is at: {tracker.get_object_location(Tracker.player2_id)}")
            time.sleep(2)
        tracker.stop_tracking()
    except Exception as e:
        print(e)
        tracker.stop_tracking()
