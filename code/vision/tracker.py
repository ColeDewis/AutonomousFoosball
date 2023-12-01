"""
NOTE: may have to undistort image before doing anything else in the main loop
(if I can get better calibration results, but that's an optimization week thing)
"""

import cv2 as cv
import numpy as np
import os
import threading
import time
if __name__ == "__main__":
    from ball_trajectory import BallTrajectory
else:   
    from vision.ball_trajectory import BallTrajectory

# scuffed way of importing from same level package since ".." wouldn't work
from pathlib import Path
import sys
_parent_dir = Path(__file__).parent.parent.resolve()
sys.path.insert(0, str(_parent_dir))
from utils.decay_functions import ExponentialDecay
from utils.weighted_moving_average import WeightedMovingAverage
sys.path.remove(str(_parent_dir))

class Tracker:
    """class for handling object detection and tracking functionality"""

    def __init__(self, cam_index: int = 0, img_w: int = 1280, img_h: int = 720, img_scale: int = 1):
        self.ball_trajectory = BallTrajectory(img_scale=img_scale, capacity=10)

        # set up the camera
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
            flag, _ = self.vc.read()

        if flag:
            self.flag = threading.Event()
            self.thread = threading.Thread(target=self.track_objects, args=(self.flag,))
            self.thread.start()
        else:
            print("[ERROR] Could not open video capture")
            self.vc.release()

    def track_objects(self, flag: threading.Event):
        """handles detecting and tracking the ball via the BallTrajectory class.
        This is the main video capture loop that is run in a separate thread

        Args:
            flag (threading.Event): a flag for telling us when to stop the video capture process
        """
        # for whatever reason, if I have this as class state python deadlocks somehow
        # this lets us output a constant sized window for the user to see
        window_name = "Gameplay"
        cv.namedWindow(window_name, cv.WINDOW_NORMAL)

        while not flag.is_set():
            # read in next frame
            rval, frame = self.vc.read()
            if not rval:
                break

            # for saving compute time. Will cause rounding errors when converting
            # coordinates back to original size but whatever
            resized_res = (self.img_w // self.img_scale, self.img_h // self.img_scale)
            resized_frame = cv.resize(frame, resized_res, interpolation=cv.INTER_AREA)
            self.ball_trajectory.step(resized_frame)

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

if __name__ == "__main__":
    # for testing
    tracker = Tracker(0)
    print("Tracker set up, object detection commencing ...")
    try:
        while tracker.thread.is_alive():
            print(f"Ball Position Estimate: {tracker.ball_trajectory.position}")
            print(f"Ball Direction Estimate: {tracker.ball_trajectory.direction}")
            print(f"Ball Speed Estimate: {tracker.ball_trajectory.speed}")
            time.sleep(2)
        tracker.stop_tracking()
    except Exception as e:
        print(e)
        tracker.stop_tracking()
