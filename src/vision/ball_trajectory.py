import time
import cv2 as cv
import numpy as np

from src.vision.camera.transforms import image_to_world, AVG_PX2CM
from src.vision.detect import detect_circles, find_optimal_circle
from src.vision.point_projection import closest_point
from src.utils.decay_functions import ExponentialDecay
from src.utils.weighted_moving_average import WeightedMovingAverage

class BallTrajectory:
    """basic best fit line trajectory model for the ball. It will hold a queue
    of detected points that are used for line fitting, and then we can use the
    unit vector obtained from this line for direction.

    NOTE: using scaled pixel coords for everything, and then unscaling and
    calling img_to_world when returning the world values (hence the attribute
    names starting with px)

    Properties:
        abrupt_change (bool): flag signalling the ball had an abrupt change in direction
        position (list | None): [x, y] world coordinates of ball position estimate
        direction (list | None): [dx, dy] world coordinates of ball direction estimate
        speed (list | None): [v_x, v_y] velocities in x and y direction of ball in world coords

    Methods:
        step(resized_frame: np.array): steps the trajectory forward in time, computing new ball information
        draw_info(frame: np.array): draws the unscaled ball information on the unscaled frame
    """

    def __init__(self, img_scale, capacity: int = 10):
        """initialize the ball trajectory with a bunch of null data and whatnot

        Args:
            img_scale (int): the amount the image was scaled down for processing
            capacity (int): the maximum number of previous positions to hold
            for building the line of best fit
        """
        # flag for checking abrupt changes in directions
        self.abrupt_change = False

        self._img_scale = img_scale
        self._prev_frame = None

        # for storing previous positions to build trajectory model
        self._capacity = capacity
        self._px_positions = []

        # last detected circle and position/direction estimates
        self._detected_circ = None
        self._px_pos_estimate = None
        self._px_dir_estimate = None

        # for getting instantaneous speed in pixel/s
        self._prev_time = time.perf_counter()
        self._px_speed = WeightedMovingAverage(ExponentialDecay, 1, self._capacity)

    @property
    def position(self) -> list | None:
        """Computes the position estimate of the ball in world coordinates
        
        Returns:
            list: [x, y] world coords of ball
        """
        if self._px_pos_estimate is not None:
            return image_to_world(*(self._px_pos_estimate * self._img_scale))
        else:
            return None

    @property
    def direction(self) -> list | None:
        """Computes the direction estimate of the ball in world frame
        
        Computes it by getting the world positions of the current image position
        as well as an image coordinate stepped along the direction vector.
        Then we get the unit direction in the world from those two world points

        Returns:
            list: [x, y] direction vector normalized in world coordinates
        """
        if self.abrupt_change:
            # need to change flag back for future .direction accesses
            self.abrupt_change = False

        if self._px_dir_estimate is not None:
            unit_dir_pt = self._px_pos_estimate + self._px_dir_estimate
            world_pos = self.position
            world_unit_dir_pt = image_to_world(*(unit_dir_pt * self._img_scale))
            world_traj = np.array(world_unit_dir_pt) - np.array(world_pos)
            return list(world_traj / np.linalg.norm(world_traj))
        else:
            return None

    @property
    def speed(self) -> float | None:
        """Computes the instantaneous speed of the ball in cm/s
        
        Returns:
            list: [v_x, v_y] velocity in each direction in cm/s
        """
        if self._px_speed.output() is not None:
            return self._px_speed.output()[0] * AVG_PX2CM # avg for now
        else:
            return None

    def step(self, resized_frame: np.array):
        """step the trajectory forward in time. This will call detection methods
        and update the position and trajectory estimate of the ball accordingly 

        Args:
            resized_frame (np.array): the scaled down image to detect objects in
        """
        if self._prev_frame is not None and np.allclose(resized_frame, self._prev_frame):
            # queried frame too fast, just return
            return

        self._prev_frame = resized_frame

        # update time interval for speed calcs
        curr_time = time.perf_counter()
        time_diff = curr_time - self._prev_time # the useful component
        self._prev_time = curr_time

        # find circles in image
        blurred = cv.edgePreservingFilter(resized_frame, cv.RECURS_FILTER, 40, 0.4)
        hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
        circles = detect_circles(hsv)

        if circles is None:
            # no ball detection case
            if self.__approximate(time_diff):
                # age trajectory so we eventually stop moving
                self._px_positions.pop(0)
            return

        # find optimal circle (ball) in image
        if len(self._px_positions) != 0:
            self._detected_circ = np.array(find_optimal_circle(circles, self._px_positions[-1]))

            # update speed estimate
            dist = np.linalg.norm(self._detected_circ[:2] - self._px_positions[-1], 2)
            self._px_speed.add_point([dist / time_diff])
        else:
            self._detected_circ = np.array(find_optimal_circle(circles))

        self._px_positions.append(self._detected_circ[:2])
        if len(self._px_positions) > self._capacity:
            self._px_positions.pop(0)

        self.__update_trajectory()

    def draw_info(self, frame: np.array):
        """helper for drawing the trajectory line and points on the window
        
        Args:
            frame (np.array): the unscaled (regular sized) frame to draw on
        """
        # draw previous circle detection
        if self._detected_circ is not None:
            scaled_circ = (self._detected_circ * self._img_scale).astype(int)
            cv.circle(frame, scaled_circ[:2], scaled_circ[2], (255, 0, 0), 2)

        # draw previous true points
        for pos in self._px_positions:
            scaled_pos = pos * self._img_scale
            cv.circle(frame, scaled_pos.astype(int), 3, (255, 0, 0), 2)

        # draw position estimate
        if self._px_pos_estimate is not None:
            scaled_pos_est = self._px_pos_estimate * self._img_scale
            cv.circle(frame, scaled_pos_est.astype(int), 3, (0, 255, 0), 2)

        # draw direction vector
        if self._px_dir_estimate is not None:
            scaled_dir_est = self._px_dir_estimate * self._img_scale
            start_pt = (scaled_pos_est - 75 * scaled_dir_est).astype(int)
            end_pt = (scaled_pos_est + 75 * scaled_dir_est).astype(int)
            cv.arrowedLine(frame, start_pt, end_pt, (255,0,0), 2)

    def __update_trajectory(self) -> tuple:
        """updates the trajectory with the latest ball detection
    
        All necessary state is updated within this function, nothing needs to
        be returned

        NOTE: we will always have at least one point in the positions list when
        we enter this function
        """
        if len(self._px_positions) < 2 or self.__is_stationary():
            # cannot reliably build trajectory estimate in this case, 
            self._px_pos_estimate = self._px_positions[-1]
            self._px_dir_estimate = None
            return
        
        if self.__changed_direction():
            # ball had an abrupt change in direction, clear old points
            self._px_positions = self._px_positions[-2:]
            self.abrupt_change = True

        vx, vy, x0, y0 = cv.fitLine(np.array(self._px_positions), cv.DIST_L2, 0, 0.01, 0.01)
        if self._px_positions[-1][0] < self._px_positions[0][0]:
            # x direction is backwards
            vx = -vx
        if (
            (vy > 0 and self._px_positions[-1][1] < self._px_positions[0][1])
            or (vy < 0 and self._px_positions[-1][1] > self._px_positions[0][1])
        ):
            # y direction is backwards, flip it
            vy = -vy

        # compute position and direction estimate from fit line and point projection
        p1 = [x0[0], y0[0]]
        p2 = [x0[0] + vx[0], y0[0] + vy[0]]
        self._px_pos_estimate = np.array(list(closest_point(list(self._px_positions[-1]), p1, p2)))
        self._px_dir_estimate = np.array([vx[0], vy[0]])

    def __approximate(self, time_diff: float) -> bool:
        """helper for computing approximate position based on direction vector
        if there was no detection of any circles

        Returns:
            bool: whether we could approximate the position based on trajectory or not
        """
        if len(self._px_positions) == 0 or self._px_dir_estimate is None:
            # literally nothing we can do at this point
            return False
        
        # just assume 50 px/s if no speed output
        if self._px_speed.output() is not None:
            speed = self._px_speed.output()[0]
        else:
            speed = 50

        self._px_pos_estimate += speed * time_diff * self._px_dir_estimate

        return True

    def __changed_direction(self, thresh: float = 15.0) -> bool:
        """Checks to see for abrupt changes in direction
        
        Args:
            thresh (float): how far the point as to be from the trajectory to
                            signal an abrupt change
        
        Returns:
            bool: if the ball change trajectory abruptly or not
        """
        if len(self._px_positions) <= 4 or self._px_dir_estimate is None:
            return False # hard to reliably tell in this scenario
        
        # find distance of last detected point from direction vector
        prev_pt = self._px_positions[-1]
        p1 = list(self._px_pos_estimate)
        p2 = list(self._px_pos_estimate + self._px_dir_estimate)
        pt_on_line = np.array(list(closest_point(list(prev_pt), p1, p2)))
        if np.sqrt(np.sum(np.square(prev_pt - pt_on_line))) > thresh:
            return True

        # fallback method for catching straight line shots (since ball would
        # still be on same trajectory just opposite direction)
        change_idx = int(3/4 * len(self._px_positions))
        before = self._px_positions[change_idx] - self._px_positions[0]
        after = self._px_positions[-1] - self._px_positions[change_idx]
        if after[1] * before[1] < 0:
            return True
        else:
            return False

    def __is_stationary(self, n: int = 5, threshold: float = 15.0) -> bool:
        """checks to see if the ball is basically stationary so we don't try
        and estimate a trajectory from its previous points.

        It does so by looking at the mean euclidean distance between the last
        n points and seeing if it's under the threshold (this is distance
        in pixels btw)

        Args:
            n (int): how many previous points to look at
            threshold (float): threshold for checking if all previous points are
                                close enough to assume ball is stationary

        Returns:
            bool: if the ball is stationary or not
        """
        total_pts = len(self._px_positions)

        if self._px_positions is None:
            return True # no points yet, assume stationary

        num_points = n if total_pts > n else total_pts
        distances = np.zeros((num_points, num_points))

        for i in range(total_pts - num_points, total_pts):
            for j in range(total_pts - num_points, total_pts):
                idx1 = i - (total_pts - num_points)
                idx2 = j - (total_pts - num_points)
                distances[idx1, idx2] = np.sqrt(np.sum((self._px_positions[i] - self._px_positions[j])**2))
        
        # Calculate the mean distance (times 2 since the matrix will double the
        # number of points we consider since its symmetric)
        mean_distance = np.sum(distances) / (2 * num_points)

        return mean_distance < threshold

if __name__ == "__main__":
    ######################### testing ##########################
    import os

    img_w, img_h = 1280, 720
    cam_index = 0
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
        scale_factor = 2

        ball_traj = BallTrajectory(img_scale=scale_factor, capacity=10)

        while True:
            # read in next frame
            rval, frame = vc.read()
            if not rval:
                break

            resized_res = (img_w // scale_factor, img_h // scale_factor)
            resized_frame = cv.resize(frame, resized_res, interpolation=cv.INTER_AREA)

            ball_traj.step(resized_frame)
            print(f"Position Estimate: {ball_traj.position}")
            print(f"Direction Estimate: {ball_traj.direction}")
            print(f"Instantaneous Speed Estimate: {ball_traj.speed}")

            ball_traj.draw_info(frame)
            cv.imshow("output", frame)

            # check if q key is pressed
            key = cv.waitKey(10)
            if key == ord('q'):
                break

    vc.release()
    cv.destroyAllWindows()
