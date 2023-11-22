import numpy as np
from time import sleep
from brick_server import BrickServer
from arduino_server import ArduinoServer
from kinematics.kinematics import Kinematics
from vision.tracker import Tracker
from utils.message_type import MessageType

class States():
    SWINGING = "SWINGING"
    TRACKING = "TRACKING"
    IDLE = "IDLE"
    DONE = "DONE"

class Robot:
    """Class to represent the robotic system as a whole, and manage the actions the 
       robot needs to take."""
    def __init__(self, brick_serv: BrickServer, arduino_serv: ArduinoServer, camera_tracker: Tracker): 
        """Initialize the robot.

        Args:
            brick_serv (BrickServer): a server to use to send messages to the brick
            arduino_serv (ArduinoServer): a server to use to send messages to an arduino
        """
        # TODO: also take in a trajectory object
        self.camera_tracker = camera_tracker
        self.brick_serv = brick_serv
        self.arduino_serv = arduino_serv
        self.kinematics = Kinematics()
        
        # intiialize initial angles for tracking position information
        self.belt_angle = 0
        self.twist_angle = 0
        self.flick_angle = 0
        
        self.state = States.IDLE
        
    def run(self):
        """Run the robot."""
        if self.state == States.IDLE:
            # Idle state for when we are waiting for an action.
            sleep(3)
            self.state = States.TRACKING
            
        elif self.state == States.TRACKING:
            # TODO: trajectory analysis to know what angle to hit at
            # TODO: use camera to get target position
            target_pos = self.camera_tracker.get_object_location(Tracker.ball_id)
            if target_pos is not None:
                target_pos = target_pos[0]
                print(target_pos)
                target_belt_angle = self.kinematics.inverse_kinematics(target_pos, 0)
                print(target_belt_angle)
                
                self.brick_serv.send_data(target_belt_angle, 0, np.pi/4, MessageType.ABSOLUTE)
                self.state = States.SWINGING
            
        elif self.state == States.SWINGING:
            # Swinging state - assumes we are at the necessary position to hit ball.
            self.__hit()
            self.state = States.DONE
        elif self.state == States.DONE:
            self.brick_serv.send_termination()
            sleep(1)
    
    def __hit(self):
        """Hit a ball."""
        self.brick_serv.send_data(0, np.pi/4, 0, MessageType.RELATIVE)
        self.brick_serv.send_data(0, -np.pi/4, 0, MessageType.RELATIVE)