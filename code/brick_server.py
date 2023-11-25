#!/usr/bin/python

"""NOTE: this code is based on given code from eClass, modified for our purposes."""

import socket
from queue import Queue
from messages.brick_message import brick_message

class BrickServer:
    """This class handles the Server side of the comunication between the laptop and the brick."""
    def __init__(self, left_host, port):
        """Initialize the server

        Args:
            host (str): host ip
            port (int): ip port
        """
        # setup server socket
        serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        #serversocket2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # We need to use the ip address that shows up in ipconfig for the usb ethernet adapter
        # that handles the comunication between the PC and the brick
        print("Setting up Server\nAddress: " + left_host + "\nPort: " + str(port))
        #print("Setting up Server\nAddress: " + left_host + "\nPort: " + str(port))
        
        serversocket.bind((left_host, port))
        #serversocket2.bind((left_host, port))
        
        # queue up to 5 requests
        serversocket.listen(5) 
        #serversocket2.listen(5)
        self.cs, addr = serversocket.accept()
        #self.cs2, addr2 = serversocket2.accept()
        self.queue = Queue()
        print(f"Connected to: {addr}")
        #print(f"Connected to: {addr2}")
        
    def send_data(self, flick: float, twist: float, speed: int, type: str):
        """Sends data to the brick client

        Args:
            flick (float): flick angle
            twist (float): twist angle
            speed (int): motor target speed
            type (MessageType): type of message
        """
        data = brick_message(flick, twist, speed, type)
        print(f"Sending Data: ({data}) to brick.")
        self.cs.send(data)
        # Waiting for the client (ev3 brick) to let the server know that it is done moving
        reply = self.cs.recv(128).decode("UTF-8")
        self.queue.put(reply)

    def send_termination(self):
        """Sends a termination message to the client. This will cause the client 
           to exit "cleanly", after stopping the motors."""
        self.cs.send("EXIT".encode("UTF-8"))