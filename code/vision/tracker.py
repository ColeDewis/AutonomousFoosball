import cv2 as cv
import numpy as np
import os
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

from ball_trajectory import BallTrajectory
from cam_info import DIST_COEF, CAM_MTX


class Tracker:
    """class for handling object detection and tracking functionality"""

    def __init__(
            self,
            cam_index: int = 0,
            img_w: int = 1280,
            img_h: int = 720,
            img_scale: int = 1,
        ):
        self.ball_trajectory = BallTrajectory(img_scale=img_scale, capacity=20)

        # set up the camera
        self.frame = None
        self.img_w = img_w
        self.img_h = img_h
        self.img_scale = img_scale
        if os.name == "nt":
            # for windows to be able to open the camera in a reasonable amount of time
            self.vc = cv.VideoCapture(cam_index, cv.CAP_DSHOW)
        else:
            self.vc = cv.VideoCapture(cam_index)
        self.vc.set(cv.CAP_PROP_FRAME_WIDTH, img_w)
        self.vc.set(cv.CAP_PROP_FRAME_HEIGHT, img_h)

        flag = False
        if self.vc.isOpened(): # try to get the first frame
            flag, self.frame = self.vc.read()

        if flag:
            # spin up threads for processing
            self.flag = threading.Event()

            self.capture_thread = threading.Thread(target=self.capture, args=(self.flag,))
            self.capture_thread.start()

            self.tracker_thread = threading.Thread(target=self.track_objects, args=(self.flag,))
            self.tracker_thread.start()
        else:
            print("[ERROR] Could not open video capture")
            self.vc.release()

    def capture(self, flag: threading.Event):
        """handles reading in and decoding images from the camera and storing them
        as class state for quicker access (since this is an IO bound op it's faster
        to throw it in a thread)

        Args:
            flag (threading.Event): a flag for telling us when to stop the video capture process
        """
        while not flag.is_set():
            # read in next frame
            rval, self.frame = self.vc.read()
            if not rval:
                flag.set() # should stop other thread

            # self.frame = cv.undistort(frame, CAM_MTX, DIST_COEF)

    def track_objects(self, flag: threading.Event):
        """handles detecting and tracking the ball via the BallTrajectory class.

        Args:
            flag (threading.Event): a flag for telling us when to stop the video capture process
        """
        # for whatever reason, if I have this as class state python deadlocks somehow.
        # this lets us output a constant sized window for the user to see
        window_name = "Gameplay"
        cv.namedWindow(window_name, cv.WINDOW_NORMAL)
        resized_res = (self.img_w // self.img_scale, self.img_h // self.img_scale)

        while not flag.is_set():
            # start = time.time()

            # copying to avoid frame changing between processing and drawing
            frame = self.frame.copy()

            # for saving compute time. Will cause rounding errors when converting
            # coordinates back to original size but whatever
            resized_frame = cv.resize(frame, resized_res, interpolation=cv.INTER_AREA)
            self.ball_trajectory.step(resized_frame)
            self.__draw_ball_information(frame)
            cv.imshow(window_name, frame)

            # check if q key is pressed
            key = cv.waitKey(10)
            if key == ord('q'):
                self.flag.set() # should stop the other thread

            # print(f"Total Loop Time: {time.time() - start}")

    def is_alive(self):
        """checks if the tracking and capture threads are still alive"""
        if not self.capture_thread.is_alive():
            return False
        if not self.tracker_thread.is_alive():
            return False
        return True

    def stop_tracking(self):
        """sends signal to any threads created by this object that they should
        quit their video capture loops and clean up
        """
        print("Stopping Tracking Process ...")
        self.flag.set()
        self.capture_thread.join()
        self.tracker_thread.join()
        self.vc.release()
        cv.destroyAllWindows()
        print("Tracking Complete")

    def __draw_ball_information(self, frame: np.array):
        """helper for drawing the trajectory line and points on the window"""
        # draw previous circle detection
        if self.ball_trajectory.detected_circ is not None:
            scaled_circ = (self.ball_trajectory.detected_circ * self.img_scale).astype(int)
            cv.circle(frame, scaled_circ[:2], scaled_circ[2], (255, 0, 0), 2)

        # draw previous true points
        for pos in self.ball_trajectory.px_positions:
            scaled_pos = pos * self.img_scale
            cv.circle(frame, scaled_pos.astype(int), 3, (255, 0, 0), 2)

        # draw position estimate
        if self.ball_trajectory.px_pos_estimate is not None:
            scaled_pos_est = self.ball_trajectory.px_pos_estimate * self.img_scale
            cv.circle(frame, scaled_pos_est.astype(int), 3, (0, 255, 0), 2)

        # draw direction vector
        if self.ball_trajectory.px_dir_estimate is not None:
            scaled_dir_est = self.ball_trajectory.px_dir_estimate * self.img_scale
            start_pt = (scaled_pos_est - 75 * scaled_dir_est).astype(int)
            end_pt = (scaled_pos_est + 75 * scaled_dir_est).astype(int)
            cv.arrowedLine(frame, start_pt, end_pt, (255,0,0), 2)

if __name__ == "__main__":
    # for testing
    tracker = Tracker(2, img_scale=2)
    print("Tracker set up, object tracking commencing ...")
    try:
        while tracker.is_alive():
            print(f"Ball Position Estimate: {tracker.ball_trajectory.position}")
            print(f"Ball Direction Estimate: {tracker.ball_trajectory.direction}")
            # print(f"Ball Speed Estimate: {tracker.ball_trajectory.speed}")
            time.sleep(2)
        tracker.stop_tracking()
    except KeyboardInterrupt:
        tracker.stop_tracking()
