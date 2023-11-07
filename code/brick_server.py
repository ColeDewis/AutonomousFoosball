#!/usr/bin/python

"""Part of the given code from eClass, modified for our purposes
"""

import socket
import time, math
from queue import Queue
from utils.message_type import MessageType

# This class handles the Server side of the comunication between the laptop and the brick.
class BrickServer:
    def __init__(self, host, port):
        """Initialize the server

        Args:
            host (str): host ip
            port (int): ip port
        """
       # setup server socket
        serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        # We need to use the ip address that shows up in ipconfig for the usb ethernet adapter that handles the comunication between the PC and the brick
        print("Setting up Server\nAddress: " + host + "\nPort: " + str(port))
        
        serversocket.bind((host, port))
        # queue up to 5 requests
        serversocket.listen(5) 
        self.cs, addr = serversocket.accept()
        print ("Connected to: " + str(addr))

    def send_angles(self, belt_angle: float, flick_angle: float, twist_angle: float, queue: Queue):
        """Send angles to brick client

        Args:
            belt_angle (float): angle to turn belt motor
            flick_angle (float): angle to turn flicking motor
            twist_angle (float): angle to twist
            queue (Queue): queue to store and return messages from client.
        """
        self.send_data(belt_angle, flick_angle, twist_angle, MessageType.ANGLES, queue)
    
    def send_position_target(self, x: float, y: float, z: float, queue: Queue):
        """Send a position target to the brick client

        Args:
            x (float): x target
            y (float): y target
            z (float): z target
            queue (Queue): queue to store and return messages from client
        """
        self.send_data(x, y, z, MessageType.POSITION, queue)
        
    def send_data(self, num1: float, num2: float, num3: float, type: str, queue: Queue):
        """Sends data to the brick client

        Args:
            num1 (float): first number (angle1 or x)
            num2 (float): second num (angle2 or y)
            num3 (float): third num (angle3 or z)
            type (MessageType): type of message
            queue (Queue): queue to store and return client messages.
        """
        data = f"{str(num1)},{str(num2)},{str(num3)},{type}" 
        print("Sending Data: (" + data + ") to robot.")
        self.cs.send(data.encode("UTF-8"))
        # Waiting for the client (ev3 brick) to let the server know that it is done moving
        reply = self.cs.recv(128).decode("UTF-8")
        queue.put(reply)

    def send_termination(self):
        """Sends a termination message to the client. This will cause the client to exit "cleanly", after stopping the motors."""
        self.cs.send("EXIT".encode("UTF-8"))