import numpy as np
import numpy.typing as nptyping
from brick_server import BrickServer
from arduino_server import ArduinoServer
from kinematics.kinematics import Kinematics
from vision.tracker import Tracker
from messages.message_type import MessageType
from utils.side import Side

class States():
    """Enum for the different states the robot can be in."""
    # initialization state
    INIT = "INIT"
    
    # swinging at a ball
    SWINGING = "SWINGING"
    
    # tracking a ball, following it
    TRACKING = "TRACKING"
    
    # moving based on points from the tracking
    MOVING = "MOVING"
    
    # after hitting, waiting for a return to be detected
    WAITING_FOR_RETURN = "WFR"
    
    # idle, waiting for new information
    IDLE = "IDLE"
    
    # routine completed, shuts down.
    DONE = "DONE"

class Robot:
    """Class to represent the robotic system as a whole, and manage the actions the 
       robot needs to take."""
    
    # Threshold at which point we should swing
    SHOULD_HIT_THRESHOLD = 7.0
    
    # TODO: determine
    BALL_Y_UPPER_BOUND = 77
    BALL_Y_LOWER_BOUND = 7
       
    def __init__(self, brick_serv: BrickServer, arduino_serv: ArduinoServer, camera_tracker: Tracker, side: Side): 
        """Initialize the robot.

        Args:
            brick_serv (BrickServer): a server to use to send messages to the brick
            arduino_serv (ArduinoServer): a server to use to send messages to an arduino
            camera_tracker (Tracker): a camera tracking object used to get object locations
            side (Side): which side this robot is on
        """
        # TODO: also take in a trajectory object
        self.camera_tracker = camera_tracker
        self.brick_serv = brick_serv
        self.arduino_serv = arduino_serv
        self.side = side
        self.kinematics = Kinematics(side)
        
        # initialize initial angles for tracking position information - we use these later to get
        # fwd kinematics
        self.belt_angle = 0
        self.belt_moving_to_angle = 0
        self.twist_angle = 0
        self.flick_angle = 0
                
        self.state = States.INIT
        
    def run(self):
        """Run the robot."""
        
        # TODO: Global check if ball is out of bounds   
        ball_pos = self.camera_tracker.ball_trajectory.position
        ball_traj = self.camera_tracker.ball_trajectory.direction
        
        # Global transition - if ball ever goes past a player, go to DONE
        if ball_pos is not None and (ball_pos[1] < self.BALL_Y_LOWER_BOUND or ball_pos[1] > self.BALL_Y_UPPER_BOUND):
            self.state = States.DONE
        
        if self.state == States.INIT:
            self.__initialize_game()
            self.state = States.IDLE
            
        elif self.state == States.IDLE:
            # Idle state for when we are waiting for an action.
            if ball_traj is None:
                return
            
            # Once the ball has a trajectory, we update state based on whether
            # the trajectory is towards us or not.
            if self.__ball_moving_towards_self(ball_traj):
                print(f"{self.side}: STARTING TRACKING")
                self.state = States.TRACKING
            else:
                print(f"{self.side}: STARTING WAITING")
                self.state = States.WAITING_FOR_RETURN
              
        elif self.state == States.TRACKING:
            if ball_pos is None:
                return
            
            target_y = ball_pos[1]
            ball_within_threshold = abs(target_y - self.kinematics.ty1) < Robot.SHOULD_HIT_THRESHOLD
            print(abs(target_y - self.kinematics.ty1))
            if ball_within_threshold:
                # the ball's y is close enough that we should just swing now
                self.state = States.SWINGING
            else:
                self.__follow_ball(ball_pos)
                if self.__done_moving():
                    self.belt_angle = self.belt_moving_to_angle
                    
        elif self.state == States.MOVING:
            # TODO: generalize for 2 systems
            if ball_pos is None:
                return
            
            target_y = ball_pos[1]
            print(target_y)
            ball_within_threshold = abs(target_y - self.kinematics.ty1) < Robot.SHOULD_HIT_THRESHOLD
            robot_done_moving = not self.arduino_serv.is_moving[self.side]
            if ball_within_threshold:
                # the ball's y is close enough that we should just swing now, even if 
                # we are still moving
                self.state = States.SWINGING
            elif robot_done_moving:
                self.state = States.TRACKING
            
        elif self.state == States.WAITING_FOR_RETURN:
            if ball_traj is not None and self.__ball_moving_towards_self(ball_traj):
                self.state = States.TRACKING
            else:
                self.__move_to_midpoint()
                if self.__done_moving():
                    self.belt_angle = self.belt_moving_to_angle
            
        elif self.state == States.SWINGING:
            # Swinging state - assumes we are at the necessary position to hit ball.
            self.__swing()
            self.state = States.WAITING_FOR_RETURN
            
        elif self.state == States.DONE:
            self.__shutdown()
            exit(0)
    
    def __initialize_game(self):
        """Initialize the robot for the game by moving it to the center.

           This blocks until initialization is done.
        """
        self.__move_to_midpoint()
        # wait for movement to stop
        while self.arduino_serv.is_moving[self.side]: pass
        self.belt_angle = self.belt_moving_to_angle
        
    def __ball_moving_towards_self(self, ball_dir: list) -> bool:
        """Checks if the ball is currently moving towards the player."""
        if self.side == Side.LEFT:
            return ball_dir[1] > 0
        elif self.side == Side.RIGHT:
            return ball_dir[1] < 0
    
    def __move_to_midpoint(self):
        """Moves the robot to the middle of the field."""
        midpt_angle = self.kinematics.inverse_kinematics(25.0, 0)
        self.arduino_serv.send_angle(midpt_angle, 10, MessageType.ABSOLUTE, self.side)
        self.belt_moving_to_angle = midpt_angle
    
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
            list: the x,y,z coordinates of the robot
        """
        return self.kinematics.forward_kinematics(self.belt_angle, self.flick_angle, self.twist_angle)
    
    def __follow_ball(self, ball_pos):
        """Follows the ball by moving to its current x position."""
        target_x = ball_pos[0] - 3
        target_belt_angle = self.kinematics.inverse_kinematics(target_x, 0)
        
        # TODO: dynamic speed based on how far we are from the ball 
        # TODO: not sure if belt angle will update properly when we stopped (it might though)
        self.belt_moving_to_angle = target_belt_angle
        self.arduino_serv.send_angle(target_belt_angle, 10, MessageType.ABSOLUTE, self.side)
        self.brick_serv.send_data(np.pi/4, 0, 30, MessageType.ABSOLUTE, self.side)
        self.flick_angle += np.pi/4
        self.twist_angle += 0     # change to hit angle we get from trajectory
    
    def __swing(self):
        """Hit a ball."""
        self.brick_serv.send_data(0, 0, 50, MessageType.ABSOLUTE, self.side)
        self.flick_angle -= np.pi/4
        
    def __shutdown(self):
        """Shuts down by sending termination to all clients."""
        self.arduino_serv.send_reset()
        self.brick_serv.send_termination()