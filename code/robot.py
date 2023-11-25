import numpy as np
from time import sleep, perf_counter
from brick_server import BrickServer
from arduino_server import ArduinoServer
from kinematics.kinematics import Kinematics
from vision.tracker import Tracker
from messages.message_type import MessageType

class States():
    SWINGING = "SWINGING"
    TRACKING = "TRACKING"
    MOVING = "MOVING"
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
            print("STARTING TRACKING STATE")
            self.state = States.TRACKING
            
        elif self.state == States.TRACKING:
            
            # TODO: trajectory analysis to know what angle to hit at
            target_pos = self.camera_tracker.get_object_location(Tracker.ball_id)
            if target_pos is not None:
                target_pos = target_pos[0]
                print(target_pos)
                target_belt_angle = self.kinematics.inverse_kinematics(target_pos, 0)
                print(target_belt_angle)
                
                self.arduino_serv.send_angle(target_belt_angle, MessageType.ABSOLUTE, ArduinoServer.RIGHT_STEPPER_ID)
                self.brick_serv.send_data(np.pi/4, 0, 30, MessageType.ABSOLUTE)
                print("STARTING MOVING STATE")
                self.state = States.MOVING
                
        elif self.state == States.MOVING:
            # TODO: generalize for 2 systems
            if not self.arduino_serv.is_moving[ArduinoServer.RIGHT_STEPPER_ID]:
                print("STARTING SWINGING STATE")
                self.state = States.SWINGING
            
        elif self.state == States.SWINGING:
            # Swinging state - assumes we are at the necessary position to hit ball.
            self.__hit()
            self.state = States.DONE
            print("STARTING DONE STATE")
            
        elif self.state == States.DONE:
            self.arduino_serv.send_reset()
            self.brick_serv.send_termination()
            sleep(1)
    
    def __hit(self):
        """Hit a ball."""
        self.brick_serv.send_data(-np.pi/4, 0, 80, MessageType.RELATIVE)