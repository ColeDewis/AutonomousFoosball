import numpy as np
import numpy.typing as nptyping
from time import perf_counter
from brick_server import BrickServer
from arduino_server import ArduinoServer
from shared_data import SharedData
from kinematics.kinematics import Kinematics
from trajectory.trajectory import Trajectory
from vision.tracker import Tracker
from messages.message_type import MessageType
from utils.weighted_moving_average import WeightedMovingAverage
from utils.decay_functions import ExponentialDecay
from utils.side import Side

class States():
    """Enum for the different states the robot can be in."""
    # initialization state
    INIT = "INIT"
    
    # swinging at a ball
    SWINGING = "SWINGING"
    
    # tracking a ball, following it
    TRACKING = "TRACKING"
    
    # winding up - we have this as a separate state to reduce delay from continously
    # sending data to bricks
    WIND_UP = "WINDUP"
    
    # after hitting, waiting for a return to be detected
    WAITING_FOR_RETURN = "WFR"
    
    # idle, waiting for new information
    IDLE = "IDLE"
    
    # routine completed, shuts down.
    DONE = "DONE"

class Robot:
    """Class to represent the robotic system as a whole, and manage the actions the 
       robot needs to take."""
    
    # max no. of seconds with no detection before we exit
    MAX_NO_DETECTION_THRESHOLD = 3
    
    # Threshold at which point we should swing
    SHOULD_HIT_THRESHOLD = 10.0 # 12
   
    # Thresholds at which the ball is out of bounds
    BALL_Y_UPPER_BOUND = 76
    BALL_Y_LOWER_BOUND = 6
       
    def __init__(
        self, 
        brick_serv: BrickServer, 
        arduino_serv: ArduinoServer, 
        camera_tracker: Tracker,
        trajectory_planner: Trajectory,
        shared_data: SharedData, 
        side: Side): 
        """Initialize the robot.

        Args:
            brick_serv (BrickServer): a server to use to send messages to the brick
            arduino_serv (ArduinoServer): a server to use to send messages to an arduino
            camera_tracker (Tracker): a camera tracking object used to get object locations
            shared_data (SharedData): object thatholds the information that the two robots share 
                                      between each other/send to each other
            side (Side): which side this robot is on
        """
        self.camera_tracker = camera_tracker
        self.brick_serv = brick_serv
        self.arduino_serv = arduino_serv
        self.shared_data = shared_data
        self.trajectory_planner = trajectory_planner
        self.side = side
        self.kinematics = Kinematics(side)
        self.trajectory_wma = WeightedMovingAverage(ExponentialDecay, 2)
        
        # initialize initial angles for tracking position information - we use these later to get
        # fwd kinematics
        self.belt_angle = 0
        self.belt_moving_to_angle = 0
        self.twist_angle = 0
        self.flick_angle = 0
        
        # state variables so we can find when we lose track of the ball for a DONE transition        
        self.last_new_detection = None
        self.last_ball_pos = None
        
        self.state = States.INIT
        
    def run(self):
        """Run the robot."""
        
        if self.camera_tracker.ball_trajectory.abrupt_change:
            self.trajectory_wma.reset()
            
        ball_pos = self.camera_tracker.ball_trajectory.position
        ball_traj = self.camera_tracker.ball_trajectory.direction
        
        # global check - if we lose detections for too long, go to DONE
        if self.state != States.IDLE and self.state != States.INIT and ball_pos == self.last_ball_pos:
            if perf_counter() - self.last_new_detection > self.MAX_NO_DETECTION_THRESHOLD:
                self.state = States.DONE
        else:
            self.last_new_detection = perf_counter()
            self.last_ball_pos = ball_pos
        
        # Global transition - if ball ever goes past a player, go to DONE
        if ball_pos is not None and (ball_pos[1] < self.BALL_Y_LOWER_BOUND or ball_pos[1] > self.BALL_Y_UPPER_BOUND):
            self.state = States.DONE
            
        # global check - if we are done moving update kinematic position
        if self.__done_moving():
            self.belt_angle = self.belt_moving_to_angle
            with self.shared_data.lock:
                self.shared_data.pos_dict[self.side] = self.__get_position()
        
        if self.state == States.INIT:
            # Move the robots to the midpoint
            self.__initialize_game()
            self.state = States.IDLE
            
        elif self.state == States.IDLE:
            # Idle state for when we are waiting for an action.
            if ball_traj is None:
                return
            
            # Once the ball has a trajectory, we update state based on whether
            # the trajectory is towards us or not.
            self.state = States.TRACKING if self.__ball_moving_towards_self(ball_traj) else States.WAITING_FOR_RETURN
              
        elif self.state == States.TRACKING:
            if ball_pos is None or ball_traj is None:
                return

            target_y = ball_pos[1]
            ball_within_threshold = abs(target_y - self.kinematics.ty1) < Robot.SHOULD_HIT_THRESHOLD
            if ball_within_threshold:
                # the ball's y is close enough that we should just swing now
                self.state = States.SWINGING
            else:
                # NORMAL: Follow expected trajectories
                self.trajectory_wma.age_points()
                self.trajectory_wma.add_point(ball_traj)
                self.__follow_expected_trajectory(ball_pos, self.trajectory_wma.output())

                # ALT: We can run this to follow the ball instead of trajectories
                #self.__follow_ball(ball_pos, ball_traj)
                
            # if, while the ball is moving towards us, it stops doing so, go to waiting for return,
            # so that we don't end up with both paddles in tracking state.
            if not self.__ball_moving_towards_self(ball_traj):
                self.state = States.WAITING_FOR_RETURN
                return
            
        elif self.state == States.WIND_UP:
            # move the flick motor backwards
            self.brick_serv.send_data(np.pi/4, 0, 30, MessageType.ABSOLUTE, self.side)
            self.flick_angle = np.pi/4
            self.flick_angle = 0
            self.state = States.WAITING_FOR_RETURN
            
        elif self.state == States.WAITING_FOR_RETURN:
            # as soon as ball starts moving towards us, start tracking (following) it
            if ball_traj is not None and self.__ball_moving_towards_self(ball_traj):
                self.state = States.TRACKING
            
        elif self.state == States.SWINGING:
            if ball_pos is None or ball_traj is None:
                return
            
            # Swinging state - assumes we are at the necessary position to hit ball.
            self.__swing(ball_pos, ball_traj)
            self.state = States.WIND_UP
            
        elif self.state == States.DONE:
            # reset all robots to their original positions
            self.__shutdown()
            exit(0)
    
    def __initialize_game(self):
        """Initialize the robot for the game by moving it to the center.

           This blocks until initialization is done.
        """
        self.brick_serv.send_data(np.pi/4, 0, 30, MessageType.ABSOLUTE, self.side)
        self.flick_angle = np.pi/4
        self.twist_angle = 0
        self.__move_to_midpoint()
        # wait for movement to stop
        while self.arduino_serv.is_moving[self.side]: pass
        self.belt_angle = self.belt_moving_to_angle
        with self.shared_data.lock:
            self.shared_data.pos_dict[self.side] = self.__get_position()
        
    def __ball_moving_towards_self(self, ball_dir: list) -> bool:
        """Checks if the ball is currently moving towards the player."""
        if self.side == Side.LEFT:
            return ball_dir[1] > 0
        elif self.side == Side.RIGHT:
            return ball_dir[1] < 0
    
    def __move_to_midpoint(self):
        """Moves the robot to the middle of the field."""
        midpt_angle = self.kinematics.inverse_kinematics(25.0, 0)
        self.belt_moving_to_angle = midpt_angle
        self.arduino_serv.send_angle(midpt_angle, 50, MessageType.ABSOLUTE, self.side)
    
    def __done_moving(self) -> bool:
        """Checks if the robot is done moving.
        
        Returns:
            bool: depending on if the robot is done moving.
        """
        return self.arduino_serv.is_moving[self.side]
    
    def __get_position(self) -> nptyping.ArrayLike:
        """Gets the position of the robot. If the robot is moving, it will give
           the position that the robot was at before movement started.

        Returns:
            ArrayLike: the x,y,z coordinates of the robot
        """
        return np.transpose(
            self.kinematics.forward_kinematics(self.belt_angle, self.flick_angle, self.twist_angle)
        )[0]
    
    def __follow_ball(self, ball_pos: list, ball_traj: list):
        """Follows the ball by moving to its current x position, with an adjustment for trajectory.

        Args:
            ball_pos (list): x, y ball position
            ball_traj (list): ball trajectory vector
        """
        # adjust our target based on the ball trajectory - that is, we overshoot the ball's position
        # based on the direction is is moving, scaling down this overshooting as the ball gets closer
        # to the y line our robot is on.
        x_trajectory = ball_traj[0]
        y_diff = abs(ball_pos[1] - self.kinematics.ty1)
        trajectory_adjustment = x_trajectory * (y_diff / 5) # TODO: this is really arbitrary, test more 
        target_x = ball_pos[0] + trajectory_adjustment
        
        target_belt_angle = self.kinematics.inverse_kinematics(target_x, 0)
        
        self.belt_moving_to_angle = target_belt_angle
        self.arduino_serv.send_angle(target_belt_angle, 100, MessageType.ABSOLUTE, self.side)
    
    def __follow_expected_trajectory(self, ball_pos: list, ball_traj: list):
        """Follow the expected trajectory of the ball by moving to where the trajectory
           predicts the ball will be when it is on the robot's y line.

        Args:
            ball_pos (list): x, y ball position
            ball_traj (list): ball trajectory vector
        """
        target_x = self.trajectory_planner.position_on_goal_line(ball_pos, ball_traj, "", self.side)
        target_belt_angle = self.kinematics.inverse_kinematics(target_x[0], 0)
            
        self.belt_moving_to_angle = target_belt_angle
        self.arduino_serv.send_angle(target_belt_angle, 100, MessageType.ABSOLUTE, self.side)
    
    def __swing(self, ball_pos: list, ball_traj: list):
        """Hit a ball, using trajectory prediction to find an optimal angle.
        
        Args:
            ball_pos (list): x, y ball position
            ball_traj (list): ball trajectory vector
        """
        # determine the angle we want to hit the ball at
        opp_side = Side.LEFT if self.side == Side.RIGHT else Side.RIGHT
        with self.shared_data.lock:
            opponent_position = self.shared_data.pos_dict[opp_side]
        hit_angle = self.trajectory_planner.find_best_angle(opponent_position[0], self.__get_position()[:2], "TOGETHER", opp_side)   
        hit_angle *= (np.pi / 180)
        
        # now that we know hit angle, send one more belt target to account for the angle we hit at
        # this is nonblocking, but the movement should be so small that it shouldn't be an issue
        target_x = self.trajectory_planner.position_on_goal_line(ball_pos, ball_traj, "", self.side)
        target_belt_angle = self.kinematics.inverse_kinematics(target_x[0], hit_angle)
        self.belt_moving_to_angle = target_belt_angle
        self.arduino_serv.send_angle(target_belt_angle, 100, MessageType.ABSOLUTE, self.side)
        
        # swing and hit the ball
        self.brick_serv.send_data(0, hit_angle, 30, MessageType.ABSOLUTE, self.side)
        self.flick_angle = 0
        self.twist_angle = hit_angle
        
    def __shutdown(self):
        """Shuts down by sending termination to all clients."""
        self.arduino_serv.send_reset()
        self.brick_serv.send_termination()