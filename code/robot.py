import numpy as np
from time import sleep
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
    
    # tracking a ball
    TRACKING = "TRACKING"
    
    # moving based on points from the tracking
    MOVING = "MOVING"
    
    # idle, waiting for new information
    IDLE = "IDLE"
    
    # routine completed, shuts down.
    DONE = "DONE"

class Robot:
    """Class to represent the robotic system as a whole, and manage the actions the 
       robot needs to take."""
    
    # Threshold at which point we should swing
    SHOULD_HIT_THRESHOLD = 15.0
       
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
        self.kinematics = Kinematics()
        
        # initialize initial angles for tracking position information
        self.belt_angle = 0
        self.twist_angle = 0
        self.flick_angle = 0
                
        self.state = States.INIT
        
    def run(self):
        """Run the robot."""
        if self.state == States.INIT:
            # initalization state, moves us to the midpoint
            midpt_angle = self.kinematics.inverse_kinematics(25.0, 0)
            self.arduino_serv.send_angle(midpt_angle, MessageType.ABSOLUTE, self.side)
            # wait for movement to stop
            while self.arduino_serv.is_moving[self.side]: pass
            
            self.state = States.IDLE
            
        elif self.state == States.IDLE:
            # Idle state for when we are waiting for an action.
            self.state = States.TRACKING
              
        elif self.state == States.TRACKING:
            target_pos = self.camera_tracker.get_object_location(Tracker.ball_id)
            if target_pos is None:
                return
            
            target_y = target_pos[1]
            print(target_y)
            ball_within_threshold = abs(target_y - self.kinematics.ty1) < Robot.SHOULD_HIT_THRESHOLD
            if ball_within_threshold:
                # the ball's y is close enough that we should just swing now and hope
                self.state = States.SWINGING
            else:
                # TODO: trajectory analysis to know what angle to hit at
                # TODO: target moving position should come from expected trajectory,
                # not from the ball's current position
                target_x = target_pos[0] - 2
                target_belt_angle = self.kinematics.inverse_kinematics(target_x, 0)
                
                self.arduino_serv.send_angle(target_belt_angle, MessageType.ABSOLUTE, self.side)
                self.brick_serv.send_data(np.pi/4, 0, 30, MessageType.ABSOLUTE, self.side)
                    
        elif self.state == States.MOVING:
            # TODO: generalize for 2 systems
            target_pos = self.camera_tracker.get_object_location(Tracker.ball_id)
            if target_pos is None:
                return
            
            target_y = target_pos[1]
            print(target_y)
            ball_within_threshold = abs(target_y - self.kinematics.ty1) < Robot.SHOULD_HIT_THRESHOLD
            robot_done_moving = not self.arduino_serv.is_moving[self.side]
            if ball_within_threshold:
                # the ball's y is close enough that we should just swing now, even if 
                # we are still moving
                self.state = States.SWINGING
            elif robot_done_moving:
                self.state = States.TRACKING
            
        elif self.state == States.SWINGING:
            # Swinging state - assumes we are at the necessary position to hit ball.
            self.__hit()
            self.state = States.DONE
            
        elif self.state == States.DONE:
            self.arduino_serv.send_reset()
            self.brick_serv.send_termination()
            sleep(1)
    
    def __hit(self):
        """Hit a ball."""
        self.brick_serv.send_data(-np.pi/4, 0, 100, MessageType.RELATIVE, self.side)