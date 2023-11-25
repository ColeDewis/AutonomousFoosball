import serial
import time
import threading
import numpy as np

from messages.message_type import MessageType
from messages.serial_message import serial_message

class ArduinoServer:
    RESET_ID = 0
    LEFT_STEPPER_ID = 1
    RIGHT_STEPPER_ID = 2
    """Server class for communicating with arduino clients."""
    def __init__(self, port: str, baudrate: int):
        """Initialize an arduino server

        Args:
            port (str): serial port, i.e. COM6
            baudrate (int): baudrate to use for serial connection
        """
        self.arduino = serial.Serial(port=port, baudrate=baudrate, timeout=None)
        self.step_pos = 0
        self.is_moving = {self.LEFT_STEPPER_ID: False, self.RIGHT_STEPPER_ID: False}
        self.STEPS_PER_REVOLUTION = 200
    
    def send_angle(self, angle: float, type: MessageType, stepper_id: int):
        """Send a target to the arduino motor.

        Args:
            angle (float): angle to move nema motor.
            type (MessageType): whether to send relative or absolute
            stepper_id (int): the int id of the motor to send to.
        """
        num_revolutions = angle / (2 * np.pi)
        steps_needed = int(self.STEPS_PER_REVOLUTION * num_revolutions)
        if type == MessageType.ABSOLUTE:
            target = steps_needed
            self.step_pos = target
        elif type == MessageType.RELATIVE:
            target = self.step_pos + steps_needed
            self.step_pos += steps_needed
        else:
            raise ValueError("Invalid message type")
        
        message = serial_message(stepper_id, target)
        self.arduino.write(message)
        self.is_moving[stepper_id] = True
        print("wrote to serial: ", message)
        
        wait_thread = threading.Thread(target=self.__wait_for_response, args=(stepper_id,))
        wait_thread.start()
    
    def __wait_for_response(self, stepper_id):
        """Asynchronously wait for a "done" response from the stepper

        Args:
            stepper_id (int): id to wait for a "done" message from
        """
        while self.is_moving[stepper_id]:
            retmsg = self.arduino.read(1)
            print(retmsg)
            if int.from_bytes(retmsg) == stepper_id:
                self.is_moving[stepper_id] = False
        
    def send_reset(self):
        """Send a message telling the arduino to reset motors."""
        message = serial_message(0, 0)
        print("wrote to serial: ", message.packet)
        self.arduino.write(message.packet)

if __name__ == "__main__":
    server = ArduinoServer("COM6", 115200)
    time.sleep(3)
    server.send_angle(np.pi, MessageType.RELATIVE, 2)
    while True:
        retmsg = server.arduino.read(1)
        print(retmsg)
        print(int.from_bytes(retmsg))
    