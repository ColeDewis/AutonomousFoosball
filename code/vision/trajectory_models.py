"""Module containing the base abstract class for a trajectory model, as well
as all models implementing the interface

The Trajectory classes should be in charge of holding the current position (or
estimated one) of an object, as well as the estimated trajectory of it

NOTE: for line of best fit trajectories, use cv.fitLine to get the unit vector
describing the direction of the line, and then we can get the angle between
that and the y-axis of our arena using the dot product between our direction
unit vector and the vector [0, 1]
"""

import time
import cv2 as cv
import numpy as np
from abc import ABC, abstractmethod

from cam_info import BALL_RADIUS_PX, PLAYER_SIDE_LEN_PX
from detect import detect_circles, detect_rectangles

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
    def get_position(self) -> list:
        """Query the position estimate of the tracked object."""
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

    NOTE: use pixel coords for everything, let img_to_world be called outside of
    the trajectory stuff potentially, lets us completely work in pixel coords in here
    """

    def __init__(self, capacity: int = 10):
        """initialize the ball trajectory with a bunch of null data and whatnot

        Args:
            capacity (int): the maximum number of previous positions to hold
            for building the line of best fit
        """
        self.capacity = capacity
        self.positions = []

        self.start_time = time.time()
        self.prev_time = self.start_time()

        self.speed = 1. # assume 1 px/sec at start, will change once we get some detections

    @property
    def prev_position(self):
        return self.positions[-1][:2] # xy-coord in image space

    def step(self, frame: np.array, hsv: np.array):
        """step the trajectory forward in time. This will call detection methods
        and update the position and trajectory estimate of the ball accordingly 

        Args:
            frame (np.array): the original (or resized) image frame for drawing on
            hsv (np.array): the blurred and color space transformed frame for color detection
        """
        circles = detect_circles(hsv)
        best_circle = self.__find_optimal_circle(circles)

        if best_circle is None:
            # nothing to update here, return
            return
        
        x, y, r = best_circle
        if self.prev_position is None:
            # no detections yet, handle separately
            self.prev_time = time.time() - self.prev_time()
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
        cv.circle(frame, (int(x), int(y)), int(r), (255, 0, 0), 2)

    def get_position(self) -> list:
        """Query the position estimate of the tracked object"""
        if self.prev_position is None:
            # no points detected so far
            return None
        else:
            # TODO: handle when last detection was too long ago, then we need
            # to calculate trajectory and estimate based speed. and then
            # handle normal case as well
            pass

    def get_trajectory(self) -> list:
        """Query the trajectory estimate of the tracked object"""
        # TODO: build line of best fit using cv2.fitLine and then get angles and
        # directions
        pass
    
    def __find_optimal_circle(self, circles: np.array) -> list | None:
        # find the closest matched circle in case multiple are detected
        circles = np.squeeze(circles)
        if circles.ndim == 1:
            circles = [circles] # stupid bug fix (for when just one circle)

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
                # and radius (using euclidean squared distance)
                dist = np.sum(np.square(circ - self.prev_position))
                if dist < min_dist:
                    min_dist = dist
                    best = circ

        return best[:2] # just want xy-coord
