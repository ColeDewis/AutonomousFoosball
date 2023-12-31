import cv2 as cv
import os
import threading
import time

from src.vision.ball_trajectory import BallTrajectory

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
            # copying to avoid frame changing between processing and drawing
            frame = self.frame.copy()

            # downscaling to lower compute time. Will cause rounding errors when
            # converting coordinates back to original size but whatever
            resized_frame = cv.resize(frame, resized_res, interpolation=cv.INTER_AREA)
            self.ball_trajectory.step(resized_frame)
            self.ball_trajectory.draw_info(frame)
            cv.imshow(window_name, frame)
            
            # check if q key is pressed
            key = cv.waitKey(10)
            if key == ord('q'):
                self.flag.set() # should stop the other thread

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

if __name__ == "__main__":
    # for testing
    tracker = Tracker(0, img_scale=2)
    print("Tracker set up, object tracking commencing ...")
    try:
        while tracker.is_alive():
            print(f"Ball Position Estimate: {tracker.ball_trajectory.position}")
            print(f"Ball Direction Estimate: {tracker.ball_trajectory.direction}")
            print(f"Ball Speed Estimate: {tracker.ball_trajectory.speed}")
            time.sleep(0.5)
        tracker.stop_tracking()
    except KeyboardInterrupt:
        tracker.stop_tracking()
