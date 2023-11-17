import numpy as np
from time import sleep
from brick_server import BrickServer
from arduino_server import ArduinoServer
from kinematics.kinematics import Kinematics
from utils.message_type import MessageType

class States():
    SWINGING = "SWINGING"
    TRACKING = "TRACKING"
    IDLE = "IDLE"

class Robot:
    """Class to represent the robotic system as a whole, and manage the actions the 
       robot needs to take."""
    def __init__(self, brick_serv: BrickServer, arduino_serv: ArduinoServer): 
        """Initialize the robot.

        Args:
            brick_serv (BrickServer): a server to use to send messages to the brick
            arduino_serv (ArduinoServer): a server to use to send messages to an arduino
        """
        # TODO: also take in a camera object
        # TODO: also take in a trajectory object
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
            target_belt_angle = self.kinematics.inverse_kinematics(12.0, np.pi/4)
            self.brick_serv.send_data(target_belt_angle, 0, np.pi/4)
            self.state = States.SWINGING
            
        elif self.state == States.SWINGING:
            # Swinging state - assumes we are at the necessary position to hit ball.
            self.__hit()
            self.state = States.IDLE
    
    def __hit(self):
        """Hit a ball."""
        self.brick_serv.send_data(0, np.pi/4, 0, MessageType.RELATIVE)
        self.brick_serv.send_data(0, -np.pi/4, 0, MessageType.RELATIVE)