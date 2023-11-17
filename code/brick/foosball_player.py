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
from utils.message_type import MessageType
from utils.util_funcs import rad2deg


class FoosballPlayer:
    belt_idx = 0
    flick_idx = 1
    twist_idx = 2

    def __init__(self):
        """Initialize a Foosball Player."""
        self.twist_motor = MediumMotor(OUTPUT_A)
        self.flick_motor = LargeMotor(OUTPUT_C)
        self.belt_motor = LargeMotor(OUTPUT_B)

        self.twist_angle = 0
        self.flick_angle = 0
        self.belt_angle = 0
        pass

    def parse_data(self, data: str):
        """Parses the data received via the socket, and determines the appropriate action.

            Data is in the form angle1, angle2, angle3, "action", where action is either "ABSOLUTE"
            or "RELATIVE", depending on movement type.

        Args:
            data (str): data string recieved on socket
        """
        input = data.split(",")

        type = input[3]
        nums = list(map(float, input[:3]))
        if type == MessageType.RELATIVE:
            self.move_relative(nums)
        elif type == MessageType.ABSOLUTE:
            self.move_absolute(nums)
        else:
            raise ValueError("Invalid message type passed")

    def move_absolute(self, angles: list):
        """Moves the motors by the given angle array, as an absolute goal 

        Args:
            angles (list): list of angles to move
        """
        self.belt_motor.on_for_degrees(SpeedPercent(10), rad2deg(angles[FoosballPlayer.belt_idx] - self.belt_angle))
        self.flick_motor.on_for_degrees(SpeedPercent(10), rad2deg(angles[FoosballPlayer.flick_idx] - self.flick_angle))
        self.twist_motor.on_for_degrees(SpeedPercent(10), rad2deg(angles[FoosballPlayer.twist_idx] - self.twist_angle))

        
        self.belt_angle = angles[FoosballPlayer.belt_idx]
        self.flick_angle = angles[FoosballPlayer.flick_idx]
        self.twist_angle = angles[FoosballPlayer.twist_idx]

    def move_relative(self, angles: list):
        """Moves the motors by the given angle array, relative to current position

        Args:
            angles (list): list of angles to move
        """
        self.belt_motor.on_for_degrees(SpeedPercent(10), rad2deg(angles[FoosballPlayer.belt_idx]))
        self.flick_motor.on_for_degrees(SpeedPercent(10), rad2deg(angles[FoosballPlayer.flick_idx]))
        self.twist_motor.on_for_degrees(SpeedPercent(10), rad2deg(angles[FoosballPlayer.twist_idx]))

        self.belt_angle += angles[FoosballPlayer.belt_idx]
        self.flick_angle += angles[FoosballPlayer.flick_idx]
        self.twist_angle += angles[FoosballPlayer.twist_idx]

    def reset_to_home(self):
        """Reset back to our original position."""
        self.belt_motor.on_for_degrees(SpeedPercent(10), rad2deg(-self.belt_angle))
        self.flick_motor.on_for_degrees(SpeedPercent(10), rad2deg(-self.flick_angle))
        self.twist_motor.on_for_degrees(SpeedPercent(10), rad2deg(-self.twist_angle))

    def reset_motors(self):
        """Resets our motors to consider the current angles to be 0."""
        self.belt_motor.reset()
        self.flick_motor.reset()
        self.twist_motor.reset()
