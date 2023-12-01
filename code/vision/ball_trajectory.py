import time
import cv2 as cv
import numpy as np

from cam_info import img_to_world, X_PX2CM, Y_PX2CM
from detect import detect_circles, find_optimal_circle
from point_projection import closest_point

class BallTrajectory:
    """basic best fit line trajectory model for the ball. It will hold a queue
    of detected points that are used for line fitting, and then we can use the
    unit vector obtained from this line for direction.

    NOTE: using unscaled pixel coords for everything, and then scaling and
    calling img_to_world when returning the trajectories (hence the attribute
    names starting with px)
    """

    def __init__(self, img_scale, capacity: int = 10):
        """initialize the ball trajectory with a bunch of null data and whatnot

        Args:
            img_scale (int): the amount the image was scaled down for processing
            capacity (int): the maximum number of previous positions to hold
            for building the line of best fit
        """
        self.img_scale = img_scale

        # for storing previous positions to build trajectory model
        self.capacity = capacity
        self.px_positions = [] # just using regular list of numpy arrays for easier mutability

        # last detected circle
        self.detected_circ = None

        # position and direction estimates
        self.px_pos_estimate = None
        self.px_dir_estimate = None

        # # for getting instantaneous speed in pixel/s
        # self.prev_time = time.time()
        # self.px_speed = None

    @property
    def position(self) -> list | None:
        """Returns the position estimate of the ball in world coordinates"""
        if self.px_pos_estimate is not None:
            return img_to_world(*(self.px_pos_estimate * self.img_scale))
        else:
            return None
        
    @property
    def direction(self) -> list | None:
        """Returns the direction estimate of the ball in world frame"""
        if self.px_dir_estimate is not None:
            # convert x,y and x + vx, y + vy to world coordinates
            # and then get unit vector in world
            unit_dir_pt = self.px_pos_estimate + self.px_dir_estimate
            world_pos = self.position
            world_unit_dir_pt = img_to_world(*(unit_dir_pt * self.img_scale))
            world_traj = np.array(world_unit_dir_pt) - np.array(world_pos)
            return list(world_traj / np.linalg.norm(world_traj)) # normalize to unit
        else:
            return None

    # @property
    # def speed(self) -> float | None:
    #     """Returns the instantaneous speed of the ball in cm/s"""
    #     if self.px_speed is not None:
    #         return self.px_speed * ((X_PX2CM + Y_PX2CM) / 2) # avg for now

    def step(self, frame: np.array):
        """step the trajectory forward in time. This will call detection methods
        and update the position and trajectory estimate of the ball accordingly 

        Args:
            frame (np.array): the image to detect objects in
        """
        # blurred = cv.GaussianBlur(frame, (5, 5), 0)
        blurred = cv.edgePreservingFilter(frame, cv.RECURS_FILTER, 40, 0.4)
        hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)

        circles = detect_circles(hsv)
        if circles is None:
            # nothing to update here except we want to still age our trajectory
            if len(self.px_positions) != 0:
                self.px_positions.pop(0)
            return

        # # update time interval for speed calcs
        # self.prev_time = time.time() - self.prev_time

        if len(self.px_positions) != 0:
            # find optimal circle detection and do instantaneous speed update
            self.detected_circ = np.array(find_optimal_circle(circles, self.px_positions[-1]))
            # dist = np.linalg.norm(self.detected_circ[:2] - self.px_positions[-1], 2)
            # self.px_speed = dist / self.prev_time
        else:
            # no previous points for optimal circle or speed calculations
            self.detected_circ = np.array(find_optimal_circle(circles))

        # update position queue
        self.px_positions.append(self.detected_circ[:2])
        if len(self.px_positions) > self.capacity:
            self.px_positions.pop(0)

        # updates all trajectory related state of the class
        self.__update_trajectory()

    def __update_trajectory(self) -> tuple:
        """updates the trajectory with the latest ball detection
    
        All necessary state is updated within this function, nothing needs to
        be returned

        NOTE: we will always have at least one point in the positions list when
        we enter this function
        """
        if len(self.px_positions) < 2 or self.__is_stationary():
            # cannot reliably build trajectory estimate, just use last detected
            # position for position estimate
            self.px_pos_estimate = self.px_positions[-1]
            self.px_dir_estimate = None
            return
        
        if self.__changed_direction():
            # ball bounced off wall, take two newest points for new trajectory
            self.px_positions = self.px_positions[-2:]

        # fit line through previous detections and project last detection onto
        # line for getting our position estimate and direction estimate
        vx, vy, x0, y0 = cv.fitLine(np.array(self.px_positions), cv.DIST_L2, 0, 0.01, 0.01)
        if self.px_positions[-1][0] < self.px_positions[0][0]:
            # if previous position's x is less than the first position in the
            # trajectory queue, then we are going backwards
            vx = -vx
        if (
            (vy > 0 and self.px_positions[-1][1] < self.px_positions[0][1])
            or (vy < 0 and self.px_positions[-1][1] > self.px_positions[0][1])
        ):
            # couldn't pin down the behaviour of the vector returned from fitLine,
            # so need to check if y direction is backwards (cause only sometimes its right)
            vy = -vy

        p1 = [x0[0], y0[0]]
        p2 = [x0[0] + vx[0], y0[0] + vy[0]]
        self.px_pos_estimate = np.array(list(closest_point(list(self.px_positions[-1]), p1, p2)))
        self.px_dir_estimate = np.array([vx[0], vy[0]])

    def __changed_direction(self) -> bool:
        """Checks to see for abrupt changes in direction"""
        if len(self.px_positions) <= 4:
            return False # hard to reliably tell in this scenario

        change_pt = self.px_positions[len(self.px_positions) - 3]
        after = self.px_positions[-1] - change_pt
        before = change_pt - self.px_positions[0]
        if after[0] * before[0] < 0 or after[1] * before[1] < 0:
            # just checking if they have different signs
            return True
        else:
            return False

    def __is_stationary(self, n: int = 5, threshold: float = 5.0) -> bool:
        """checks to see if the ball is basically stationary so we don't try
        and estimate a trajectory from its previous points.

        It does so by looking at the mean euclidean distance between the last
        n points and seeing if it's under the threshold (this is distance
        in pixels btw)

        Args:
            threshold (float): threshold for checking if all previous points are
                                close enough to assume ball is stationary
        """
        total_pts = len(self.px_positions)

        if self.px_positions is None:
            return True # no points yet, assume stationary

        num_points = n if total_pts > n else total_pts
        distances = np.zeros((num_points, num_points))

        for i in range(total_pts - num_points, total_pts):
            for j in range(total_pts - num_points, total_pts):
                idx1 = i - (total_pts - num_points)
                idx2 = j - (total_pts - num_points)
                distances[idx1, idx2] = np.sqrt(np.sum((self.px_positions[i] - self.px_positions[j])**2))
        
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
            # print(f"Instantaneous Speed Estimate: {ball_traj.speed}")

            # check if q key is pressed
            key = cv.waitKey(10)
            if key == ord('q'):
                break

    vc.release()
    cv.destroyAllWindows()
