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

BALL_RADIUS_PX = 50 # pixels TODO: PROPERLY MEASURE THIS
PLAYER_SIDE_LEN_PX = 100 # pixels TODO: PROPERLY MEASURE THIS

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

    def __init__(self, cam_index: int = 0):
        # TODO: update the trajectories once object tracking has been established
        # NOTE: potentially might have to just keep locations as points
        # and then handle noise when updating trajectories (depends how fast we
        # update points and potentially how much we can skew weighted average)
        self.ball_location = WeightedMovingAverage(ExponentialDecay, 2)
        self.ball_trajectory = None

        self.player1_location = WeightedMovingAverage(ExponentialDecay, 2)
        self.player1_trajectory = None

        self.player2_location = WeightedMovingAverage(ExponentialDecay, 2)
        self.player2_trajectory = None

        # set up the camera (should be fine to do this out here since the thread
        # only ever reads from the camera object)
        self.vc = cv.VideoCapture(cam_index)
        self.vc.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
        self.vc.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

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

        TODO: need to prune old points or else those lists will get massive
        (see if we can add a cap to moving average so it will prune automatically
        like a fixed size queue)

        Args:
            flag (threading.Event): a flag for telling us when to stop the video capture process
        """

        while not flag.is_set():
            # read in next frame
            rval, frame = self.vc.read()
            if not rval:
                break

            # age previous locations
            self.ball_location.age_points()
            self.player1_location.age_points()
            self.player2_location.age_points()

            ball_location = self.detect_ball(frame)
            if ball_location is not None:
                self.ball_location.add_point(ball_location)

            player1_location, player2_location = self.detect_players(frame)
            if player1_location is not None:
                self.player1_location.add_point(player1_location)
            if player2_location is not None:
                self.player2_location.add_point(player2_location)

            # shows the original image with the detected objects drawn
            cv.imshow("Result", frame)

            # check if q key is pressed
            key = cv.waitKey(20)
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

    def detect_ball(self, frame: np.array) -> list | None:
        """uses hsv color space masking and basic image processing to find
        the ball in the image frame
        
        Args:
            frame (np.array): image frame

        Returns:
            None: no ball detected
            list: [x, y] coord of center of ball
        """
        blurred = cv.medianBlur(frame, 11)
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        # TODO: test various kernel sizes and iteration numbers
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE,(7,5))
        mask = cv.inRange(hsv, RED_LOW_MASK, RED_HIGH_MASK)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel, iterations=1)
        # mask = cv.erode(mask, kernel, iterations=1)
        # mask = cv.dilate(mask, kernel, iterations=1)

        # Mask the blurred image so that we only consider the areas with the desired colour
        masked_blurred = cv.bitwise_and(blurred,blurred, mask= mask)
        gray_mask = cv.cvtColor(masked_blurred, cv.COLOR_BGR2GRAY)

        # TODO: play with parameters to find best ones
        circles = cv.HoughCircles(gray_mask, cv.HOUGH_GRADIENT, 1.5, 300, param1=100, param2=20, minRadius=20, maxRadius=200)
        if circles is None:
            return None # no circles found
        else:
            # find the closest matched circle in case multiple are detected
            min_dist = np.inf
            best = None
            prev = self.ball_location.output()
            if prev is not None:
                prev = np.append(prev, BALL_RADIUS_PX)

            circles = np.squeeze(circles)
            if circles.ndim == 1:
                circles = [circles] # stupid bug fix

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

        cv.circle(frame, (int(best[0]), int(best[1])), int(best[2]), (255, 0, 0), 4)

        return list(best[:2]) # just want x and y coords of center

    def detect_players(self, frame: np.array) -> tuple | None:
        """uses hsv color space masking and basic image processing to find
        player1 and player2 in the image frame.
        
        NOTE: player1 is the right player in the image frame, and player2 is the
        left. Both players will be of the same color for simplicity in the code

        TODO: finish square detection (or rectangle, whatever shape the colored
        pieces are)
        
        Args:
            frame (np.array): image frame

        Returns:
            tuple: the xy-coords of both player1 and player2 as ([x1, y1], [x2, y2])
                    if one player or no players found, their list is replaced
                    by None
        """
        # Uncomment for gaussian blur
        #blurred = cv.GaussianBlur(frame, (11, 11), 0)
        blurred = cv.medianBlur(frame, 11)
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        kernel = cv.getStructuringElement(cv.MORPH_RECT,(5, 5))
        mask = cv.inRange(hsv, BLUE_LOW_MASK, BLUE_HIGH_MASK)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel, iterations=1)
        # mask = cv.erode(mask, np.ones((11, 11),np.uint8), iterations=2)
        # mask = cv.dilate(mask, np.ones((11, 11),np.uint8), iterations=3)

        # Mask the blurred image so that we only consider the areas with the desired colour
        masked_blurred = cv.bitwise_and(blurred,blurred, mask= mask)
        gray_mask = cv.cvtColor(masked_blurred, cv.COLOR_BGR2GRAY)

        return None, None
    
    def get_object_location(self, obj_id: int) -> list | None:
        """query most recent location of an object. Object ID options are:
        -   Tracker.ball_id
        -   Tracker.player1_id
        -   Tracker.player2_id

        Args:
            obj_id (int): the ID of the object whose location you want

        Returns:
            None: incorrect object ID or object has no detections yet
            list: [x, y] coordinate of object in world frame
        """
        location = None
        if obj_id == Tracker.ball_id:
            ball_loc = self.ball_location.output()
            if ball_loc is not None:
                location = self._img_to_world(*ball_loc)
        elif obj_id == Tracker.player1_id:
            player1_loc = self.ball_location.output()
            if player1_loc is not None:
                location = self._img_to_world(*player1_loc)
        elif obj_id == Tracker.player2_id:
            player2_loc = self.ball_location.output()
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
        # TODO: add transform stuff here
        pass

if __name__ == "__main__":
    # for testing
    tracker = Tracker(0)
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
