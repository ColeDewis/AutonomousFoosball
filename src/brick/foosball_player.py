from ev3dev2.motor import (
    LargeMotor,
    MediumMotor,
    OUTPUT_A,
    OUTPUT_B,
    OUTPUT_C,
    OUTPUT_D,
    SpeedPercent,
)

import sys
sys.path.append("..")
from messages.message_type import MessageType
from utils.util_funcs import rad2deg


class FoosballPlayer:
    """Class to represent a Foosball Player running on an ev3 brick."""
    flick_idx = 0
    twist_idx = 1

    def __init__(self):
        """Initialize a Foosball Player."""
        self.twist_motor = MediumMotor(OUTPUT_A)
        self.flick_motor = LargeMotor(OUTPUT_B)

        self.twist_angle = 0
        self.flick_angle = 0
        pass

    def parse_data(self, data: str):
        """Parses the data received via the socket, and determines the appropriate action.

            Data is in the form angle1, angle2, "action", where action is either "ABSOLUTE"
            or "RELATIVE", depending on movement type.

        Args:
            data (str): data string recieved on socket
        """
        input = data.split(",")

        type = input[3]
        nums = list(map(float, input[:3]))
        if type == MessageType.RELATIVE:
            self.move_relative(nums[:2], nums[2])
        elif type == MessageType.ABSOLUTE:
            self.move_absolute(nums[:2], nums[2])
        else:
            raise ValueError("Invalid message type passed")

    def move_absolute(self, angles: list, speed: int):
        """Moves the motors by the given angle array, as an absolute goal 

        Args:
            angles (list): list of angles to move
            speed (int): speed to run motors at
        """
        self.flick_motor.on_for_degrees(SpeedPercent(speed), rad2deg(angles[FoosballPlayer.flick_idx] - self.flick_angle), block=False)
        self.twist_motor.on_for_degrees(SpeedPercent(speed), rad2deg(angles[FoosballPlayer.twist_idx] - self.twist_angle), block=False)
        self.flick_motor.wait_until_not_moving()
        self.twist_motor.wait_until_not_moving()
        
        self.flick_angle = angles[FoosballPlayer.flick_idx]
        self.twist_angle = angles[FoosballPlayer.twist_idx]

    def move_relative(self, angles: list, speed: int):
        """Moves the motors by the given angle array, relative to current position

        Args:
            angles (list): list of angles to move
            speed (int): speed to run motors at
        """
        self.flick_motor.on_for_degrees(SpeedPercent(speed), rad2deg(angles[FoosballPlayer.flick_idx]), block=False)
        self.twist_motor.on_for_degrees(SpeedPercent(speed), rad2deg(angles[FoosballPlayer.twist_idx]), block=False)
        self.flick_motor.wait_until_not_moving()
        self.twist_motor.wait_until_not_moving()

        self.flick_angle += angles[FoosballPlayer.flick_idx]
        self.twist_angle += angles[FoosballPlayer.twist_idx]

    def reset_to_home(self):
        """Reset back to our original position."""
        self.flick_motor.on_for_degrees(SpeedPercent(10), rad2deg(-self.flick_angle))
        self.twist_motor.on_for_degrees(SpeedPercent(10), rad2deg(-self.twist_angle))

    def reset_motors(self):
        """Resets our motors to consider the current angles to be 0."""
        self.flick_motor.reset()
        self.twist_motor.reset()
