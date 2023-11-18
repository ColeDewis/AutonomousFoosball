import serial
import time
import numpy as np

from utils.message_type import MessageType

class ArduinoServer:
    """Server class for communicating with arduino clients."""
    def __init__(self, port: str, baudrate: int):
        """Initialize an arduino server

        Args:
            port (str): serial port, i.e. COM6
            baudrate (int): baudrate to use for serial connection
        """
        self.arduino = serial.Serial(port=port, baudrate=baudrate, timeout=None)
        self.step_pos = 0
        self.STEPS_PER_REVOLUTION = 200
    
    def send_angle(self, angle: float, type: MessageType):
        """Send a target to the arduino motor.

        Args:
            angle (float): angle to move nema motor.
            type (MessageType): whether to send relative or absolute
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
        
        print(bytes(f"{target}", "utf-8"))
        self.arduino.write(bytes(f"{target}", "utf-8"))
        print("wrote: ", target)
    

if __name__ == "__main__":
    server = ArduinoServer()
    time.sleep(3)
    
    server.send_angle(2 * np.pi)
    #while True:
    print(server.arduino.readline().decode().strip())
    