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

            Data is in the form num1, num2, num3, "action", where action is either "POSITION"
            or "ANGLES", and num1, num2, num3 is then interpreted either as angles to move
            or a position target.

            If we define angles, then num1 = belt angle, num2 = flick angle, num3 = twist angle

            If we define position, we send x, y, z, though z will likely always be ignored/not sent

        Args:
            data (str): data string recieved on socket
        """
        input = data.split(",")

        type = input[3]
        nums = list(map(float, input[:3]))
        if type == MessageType.ANGLES:
            self.move_angles(nums)
        elif type == MessageType.POSITION:
            self.move_to_position(nums)
        else:
            raise ValueError("Invalid message type passed")

    def report_position(self):
        """Reports the current position of the player."""

        # TODO: define fwd kinematics to be able to do this
        pass

    def move_angles(self, angles: list):
        """Moves the motors by the given angle array.

        Args:
            angles (list): _description_
        """
        self.belt_motor.on_for_degrees(SpeedPercent(10), rad2deg(angles[FoosballPlayer.belt_idx]))
        self.flick_motor.on_for_degrees(SpeedPercent(10), rad2deg(angles[FoosballPlayer.flick_idx]))
        self.twist_motor.on_for_degrees(SpeedPercent(10), rad2deg(angles[FoosballPlayer.twist_idx]))

        self.belt_angle += angles[FoosballPlayer.belt_idx]
        self.flick_angle += angles[FoosballPlayer.flick_idx]
        self.twist_angle += angles[FoosballPlayer.twist_idx]

    def move_to_position(self, position: list):
        """Move to position with inverse kinematics

        Args:
            position (list): x, y, z target position
        """
        # TODO: Inverse Kinematics
        pass

    def reset_to_home(self):
        """Reset back to our original position."""
        self.belt_motor.on_for_degrees(SpeedPercent(10), rad2deg(-self.belt_angle))
        self.flick_motor.on_for_degrees(SpeedPercent(10), rad2deg(-self.flick_angle))
        self.twist_motor.on_for_degrees(SpeedPercent(10), rad2deg(-self.twist_angle))

    def reset_motors(self):
        self.belt_motor.reset()
        self.flick_motor.reset()
        self.twist_motor.reset()
