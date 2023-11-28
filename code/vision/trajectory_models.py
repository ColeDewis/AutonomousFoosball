"""Module containing the base abstract class for a trajectory model, as well
as all models implementing the interface

The Trajectory classes should be in charge of holding the current position (or
estimated one) of an object, as well as the estimated trajectory of it

TODO: use cv.arrowedLine() to check which direction the unit vector is always
facing. Would be sick if it always went towards the last point

NOTE: for line of best fit trajectories, use cv.fitLine to get the unit vector
describing the direction of the line, and then we can get the angle between
that and the y-axis of our arena using the dot product between our direction
unit vector and the vector [0, 1]

NOTE: for player trajectories, don't build a line or anything, literally just
return player position cause that's all we really need
"""

import time
import cv2 as cv
import numpy as np
from abc import ABC, abstractmethod

from cam_info import BALL_RADIUS_PX, PLAYER_SIDE_LEN_PX, img_to_world
from detect import detect_circles, detect_rectangles
from point_projection import closest_point

class Trajectory(ABC):
    """Base class for a trajectory model of an object in the image feed"""

    @abstractmethod
    def step(self, frame: np.array, hsv: np.array):
        """step the trajectory forward in time. This will call detection methods
        and update the position and trajectory estimate of the object accordingly

        Args:
            frame (np.array): the original (or resized) image frame for drawing on
            hsv (np.array): the blurred and color space transformed frame for color detection
        """
        pass

    @abstractmethod
    def get_trajectory(self) -> list:
        """Query the trajectory estimate of the tracked object"""
        pass

class BallLineTrajectory(Trajectory):
    """basic best fit line trajectory model for the ball. It will hold a queue
    of detected points that are used for line fitting, and then we can use the
    unit vector obtained from this line to get angle of motion. We can get the
    direction along that line by just getting the y values of the first and last
    point in the positions list and seeing if last - first is (-) or not (this
    will depend on if we use pixel coords or world coords)

    NOTE: lazy building trajectory models, so just storing previous positions
    until someone calls get_trajectory()

    NOTE: use pixel coords for everything, and then call img_to_world at the end
    and renormalize
    """

    def __init__(self, capacity: int = 10):
        """initialize the ball trajectory with a bunch of null data and whatnot

        Args:
            capacity (int): the maximum number of previous positions to hold
            for building the line of best fit
        """
        self.capacity = capacity
        self.positions = []
        self.prev_time = time.time() # NOTE: I'm not sure if speed will actually be necessary
        self.speed = 1. # assume 1 px/sec at start, will change once we get some detections

    @property
    def prev_position(self):
        if len(self.positions) != 0:
            return self.positions[-1][:2] # xy-coord in image space
        else:
            return None

    def step(self, hsv: np.array, frame: np.array = None):
        """step the trajectory forward in time. This will call detection methods
        and update the position and trajectory estimate of the ball accordingly 

        Args:
            hsv (np.array): the blurred and color space transformed frame for color detection
            frame (np.array): the original (or resized) image frame for drawing on (optional)
        """
        circles = detect_circles(hsv)
        if circles is None:
            # nothing to update here, skip step
            return

        x, y, r = self.__find_optimal_circle(circles)
        if self.prev_position is None:
            # no detections yet, can't get speed
            self.prev_time = time.time() - self.prev_time
        else:
            # speed update
            self.prev_time = time.time() - self.prev_time
            dist = np.linalg.norm(np.array([x,y]) - np.array(self.prev_position), 2)
            self.speed = dist / self.prev_time

        # update position queue
        self.positions.append([x, y])
        if len(self.positions) > self.capacity:
            self.positions.pop(0)
        
        # lastly, draw circle on image
        if frame is not None:
            cv.circle(frame, (int(x), int(y)), int(r), (255, 0, 0), 2)

    def get_trajectory(self, frame: np.array = None) -> tuple:
        """Query the trajectory estimate of the tracked object

        TODO: refresh the trajectory when we hit a wall
        TODO: FIX THE RETURN WORLD COORDINATES, THEY ARE WRONG
        
        Args:
            frame (np.array): frame for drawing trajectory on (optional)

        Returns:
            tuple: (position_estimate, direction_estimate), both in the form of
                    [x, y]. If either are nonexistent, returns None in place of
                    the list of coordinates
        """
        if len(self.positions) < 2:
            # not enough points to build a trajectory
            if self.prev_position is not None:
                return img_to_world(*self.prev_position), None
            else:
                return None, None
        
        if self.__stationary_check():
            # ball is approximately stationary, no possible trajectory
            return img_to_world(*self.prev_position), None

        # TODO: test parameters
        # fit line through previous detections and project last detection onto
        # line for getting our position estimate and direction estimate
        vx, vy, x0, y0 = cv.fitLine(np.array(self.positions), cv.DIST_L2, 0, 0.01, 0.01)
        p1 = [x0[0], y0[0]]
        p2 = [x0[0] + vx[0], y0[0] + vy[0]]
        pos_estimate = list(closest_point(self.prev_position, p1, p2))
        # NOTE: doing this because I'm not sure if transforming a vector from
        # image to world is the same as transforming a point
        unit_dir_pt = np.array(pos_estimate) + np.array([vx[0], vy[0]])

        # TODO: check if I need to flip the direction

        # convert x,y and x + vx, y + vy to world coordinates, and then get unit vector in world
        # TODO: check why these seem so off
        world_pos = img_to_world(*pos_estimate)
        world_unit_dir_pt = img_to_world(*unit_dir_pt)
        world_traj = np.array(world_unit_dir_pt) - np.array(world_pos)
        world_traj = world_traj / np.linalg.norm(world_traj) # normalize to unit

        if frame is not None:
            # draw trajectory stuff onto frame for debugging
            for pos in self.positions:
                cv.circle(frame, (int(pos[0]), int(pos[1])), 3, (255, 0, 0), 2)
            cv.circle(frame, (int(pos_estimate[0]), int(pos_estimate[1])), 3, (0, 255, 0), 2)
            cv.arrowedLine(
                frame,
                (int(pos_estimate[0] - 100 * vx[0]), int(pos_estimate[1] - 100 * vy[0])),
                (int(pos_estimate[0] + 100 * vx[0]), int(pos_estimate[1] + 100 * vy[0])),
                (255,0,0),
                2
            )

        # returning everything as lists right now, could change to np arrays later
        return list(world_pos), list(world_traj)

    def __stationary_check(self, threshold: float = 5.0) -> bool:
        """checks to see if the ball is basically stationary so we don't try
        and estimate a trajectory from its previous points
        
        Args:
            threshold (float): threshold for checking if all previous points are
                                close enough to assume ball is stationary
        """
        if self.positions is None:
            return True # no points yet, assume stationary

        # check if all points have been too close to build an accurate model
        # currently a pretty jank way of checking, could update to just be
        # if max euclidean distance between any two points is < threshold
        differences = np.diff(np.array(self.positions), axis=0)
        return np.absolute(np.mean(np.sum(differences, axis=0))) < threshold

    # TODO: put this in detection module with prev_position as an argument
    def __find_optimal_circle(self, circles: np.array) -> list:
        # find the closest matched circle in case multiple are detected
        # NOTE: don't think I need bug fix below anymore since handled in detect.py
        # circles = np.squeeze(circles)
        # if circles.ndim == 1:
        #     circles = [circles] # stupid bug fix (for when just one circle)

        min_dist = np.inf
        for circ in circles: # circ = [x, y, rad]
            if self.prev_position is None:
                # no detection to match so far, pick one with closest radius
                diff = abs(circ[2] - BALL_RADIUS_PX)
                if diff < min_dist:
                    min_dist = diff
                    best = circ
            else:
                # we have previous detections, find best match according to
                # distance from previous point (since likely very close still)
                dist = np.sum(np.square(circ[:2] - self.prev_position))
                if dist < min_dist:
                    min_dist = dist
                    best = circ

        return best

if __name__ == "__main__":
    # # testing
    # img = np.zeros((360, 640, 3), dtype=np.uint8)

    # x = np.arange(1, 6, 1)
    # y = x * 2 + 1
    # positions = np.array([(u, v) for u, v in zip(x, y)])
    # differences = np.diff(np.array(positions), axis=0)
    # print(np.sum(differences, axis=0))
    # print(np.mean(np.sum(differences, axis=0)))
    # print(img.shape[0])
    # vx, vy, x0, y0 = cv.fitLine(positions, cv.DIST_L2, 0, 0.01, 0.01)
    # print(list(closest_point([0, 2], [x0[0], y0[0]], [x0[0] + vx[0], y0[0] + vy[0]])))

    # cv.line(img, (int(x0[0] - 1000 * vx[0]), int(y0[0] - 1000 * vy[0])), (int(x0[0] + 1000 * vx[0]), int(y0[0] + 1000 * vy[0])), (255,0,0), 2)
    # cv.imshow("line", img)
    # cv.waitKey(0)
    # cv.destroyAllWindows()
    ######################### testing ##########################
    import os

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

        ball_traj = BallLineTrajectory()

        while True:
            # read in next frame
            rval, frame = vc.read()
            if not rval:
                break

            resized_res = (img_w // scale_factor, img_h // scale_factor)
            resized_frame = cv.resize(frame, resized_res, interpolation=cv.INTER_AREA)
            blurred = cv.edgePreservingFilter(resized_frame, cv.RECURS_FILTER, 40, 0.4)
            hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

            ball_traj.step(hsv, resized_frame)
            pos, traj = ball_traj.get_trajectory(resized_frame)
            print(pos, traj)
    
            # shows the original image with the detected objects drawn
            cv.imshow(window_name, resized_frame)
            cv.resizeWindow(window_name, img_w, img_h)

            # check if q key is pressed
            key = cv.waitKey(10)
            if key == ord('q'):
                break

    vc.release()
    cv.destroyAllWindows()
