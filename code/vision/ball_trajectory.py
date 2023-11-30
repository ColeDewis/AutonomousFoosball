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

        # position and direction estimates
        self.px_pos_estimate = None
        self.px_dir_estimate = None

        # for getting instantaneous speed in pixel/s
        self.prev_time = time.time()
        self.px_speed = None

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
            # TODO: check that this works
            return img_to_world(*self.px_dir_estimate, is_vec=True)
        else:
            return None

    @property
    def speed(self) -> float | None:
        """Returns the instantaneous speed of the ball in cm/s"""
        if self.px_speed is not None:
            return self.px_speed * ((X_PX2CM + Y_PX2CM) / 2) # avg for now

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

        # update time interval for speed calcs
        self.prev_time = time.time() - self.prev_time

        if len(self.px_positions) != 0:
            # find optimal circle detection and do instantaneous speed update
            x, y, r = find_optimal_circle(circles, self.px_positions[-1])
            dist = np.linalg.norm(np.array([x, y]) - self.px_positions[-1], 2)
            self.px_speed = dist / self.prev_time
        else:
            # no previous points for optimal circle or speed calculations
            x, y, r = find_optimal_circle(circles)

        # update position queue
        self.px_positions.append(np.array([x, y]))
        if len(self.px_positions) > self.capacity:
            self.px_positions.pop(0)

        # updates all trajectory related state of the class
        self.__update_trajectory()

        cv.circle(frame, (int(x), int(y)), int(r), (255, 0, 0), 2)
        self.__draw_trajectory(frame)

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
        
        if self.__check_bounce():
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

        # TODO: COMPARE THIS METHOD BELOW WITH THE HOMOGENEOUS 0 METHOD TO SEE
        # IF THEY RETURN THE SAME WORLD VECTOR
        # -------------------------------------------------------------------- #
        # # NOTE: doing this because I'm not sure if transforming a vector from
        # # image to world is the same as transforming a point
        # unit_dir_pt = np.array(pos_estimate) + np.array([vx[0], vy[0]])

        # # convert x,y and x + vx, y + vy to world coordinates, and then get unit vector in world
        # # NOTE: update this to just use homogeneous representation of direction in
        # # img_to_world instead of all these extra step
        # world_pos = img_to_world(*pos_estimate)
        # world_unit_dir_pt = img_to_world(*unit_dir_pt)
        # world_traj = np.array(world_unit_dir_pt) - np.array(world_pos)
        # world_traj = list(world_traj / np.linalg.norm(world_traj)) # normalize to unit

    def __check_bounce(self) -> bool:
        """Checks if the ball hit the wall by comparing the y-differences of the
        second half of the positions to the first (since storing in pixel coords)
        """
        if len(self.px_positions) == 2:
            return False # no way to reliably tell in this scenario
        
        mid = self.px_positions[int(len(self.px_positions) / 2)]
        after_y = self.px_positions[-1][1] - mid[1]
        before_y = mid[1] - self.px_positions[0][1]
        if after_y * before_y < 0:
            # just checking if they have different signs
            return True
        else:
            return False

    def __is_stationary(self, threshold: float = 5.0) -> bool:
        """checks to see if the ball is basically stationary so we don't try
        and estimate a trajectory from its previous points
        
        Args:
            threshold (float): threshold for checking if all previous points are
                                close enough to assume ball is stationary
        """
        if self.px_positions is None:
            return True # no points yet, assume stationary

        # check if all points have been too close to build an accurate model
        # currently a pretty jank way of checking, could update to just be
        # if max euclidean distance between any two points is < threshold
        differences = np.diff(np.array(self.px_positions), axis=0)
        return np.absolute(np.mean(np.sum(differences, axis=0))) < threshold

    def __draw_trajectory(self, frame: np.array):
            """helper for drawing trajectory stuff on the image"""
            # draw previous true points
            for pos in self.px_positions:
                cv.circle(frame, pos.astype(int), 3, (255, 0, 0), 2)

            # draw position estimate
            cv.circle(frame, self.px_pos_estimate.astype(int), 3, (0, 255, 0), 2)

            # draw direction vector
            if self.px_dir_estimate is not None:
                start_pt = (self.px_pos_estimate - 100 * self.px_dir_estimate).astype(int)
                end_pt = (self.px_pos_estimate + 100 * self.px_dir_estimate).astype(int)
                cv.arrowedLine(frame, start_pt, end_pt, (255,0,0), 2)

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
        window_name = "Testing"
        cv.namedWindow(window_name, cv.WINDOW_NORMAL)
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
    
            # shows the original image with the detected objects drawn
            cv.imshow(window_name, resized_frame)
            cv.resizeWindow(window_name, img_w, img_h)

            # check if q key is pressed
            key = cv.waitKey(10)
            if key == ord('q'):
                break

    vc.release()
    cv.destroyAllWindows()
