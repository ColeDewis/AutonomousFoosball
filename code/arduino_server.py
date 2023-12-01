import serial
import time
import threading
import numpy as np

from messages.message_type import MessageType
from messages.serial_message import serial_message

from utils.side import Side

class ArduinoServer:
    RESET_ID = 0
    LEFT_STEPPER_ID = 1
    RIGHT_STEPPER_ID = 2
    STEPS_PER_REVOLUTION = 200
    
    """Server class for communicating with arduino clients."""
    def __init__(self, port: str, baudrate: int):
        """Initialize an arduino server

        Args:
            port (str): serial port, i.e. COM6
            baudrate (int): baudrate to use for serial connection
        """
        self.arduino = serial.Serial(port=port, baudrate=baudrate, timeout=None)
        self.step_pos = {Side.LEFT: 0, Side.RIGHT: 0}
        self.side_to_id = {Side.LEFT: self.LEFT_STEPPER_ID, Side.RIGHT: self.RIGHT_STEPPER_ID}
        self.is_moving = {Side.LEFT: False, Side.RIGHT: False}
        wait_thread = threading.Thread(target=self.__wait_for_response)
        wait_thread.start()
    
    def send_angle(self, angle: float, speed: int, type: MessageType, side: Side):
        """Send a target to the arduino motor.

        Args:
            angle (float): angle to move nema motor.
            speed (int): speed to use, to be multiplied by 10. Should be from 1-13.
            type (MessageType): whether to send relative or absolute
            side (Side): the side to send the command to
        """
        num_revolutions = angle / (2 * np.pi)
        steps_needed = int(self.STEPS_PER_REVOLUTION * num_revolutions)
        if type == MessageType.ABSOLUTE:
            target = steps_needed
            self.step_pos[side] = target
        elif type == MessageType.RELATIVE:
            target = self.step_pos[side] + steps_needed
            self.step_pos[side] += steps_needed
        else:
            raise ValueError("Invalid message type")
        
        stepper_id = self.side_to_id[side]
        message = serial_message(stepper_id, speed, target)
        self.arduino.write(message)
        self.is_moving[side] = True
        print("wrote to serial: ", message)
    
    def __wait_for_response(self):
        """Asynchronously wait for a "done" response from the stepper."""
        while True:
            retmsg = self.arduino.read(1)
            if int.from_bytes(retmsg) == self.LEFT_STEPPER_ID:
                self.is_moving[Side.LEFT] = False
            elif int.from_bytes(retmsg) == self.RIGHT_STEPPER_ID:
                self.is_moving[Side.RIGHT] = False
        
    def send_reset(self):
        """Send a message telling the arduino to reset ALL motors."""
        message = serial_message(0, 0, 0) 
        print("wrote to serial: ", message)
        self.arduino.write(message)

if __name__ == "__main__":
    server = ArduinoServer("COM6", 115200)
    time.sleep(3)
    server.send_angle(8 * np.pi, 10, MessageType.ABSOLUTE, Side.RIGHT)
    server.send_angle(8 * np.pi, 10, MessageType.ABSOLUTE, Side.LEFT)
    while True:
        retmsg = server.arduino.read(1)
        print(retmsg)
        print(int.from_bytes(retmsg))
    