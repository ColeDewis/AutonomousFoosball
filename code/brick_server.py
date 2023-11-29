#!/usr/bin/python

"""NOTE: this code is based on given code from eClass, modified for our purposes."""

import socket
from queue import Queue
from messages.brick_message import brick_message
from utils.side import Side

class BrickServer:
    """This class handles the Server side of the comunication between the laptop and the brick."""
    def __init__(self, left_host: str, right_host: str, port: int):
        """Initialize the server

        Args:
            left_host (str): host ip for left brick
            right_host (str): host ip for right brick
            port (int): ip port
        """
        # setup server socket
        rightsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        #leftsocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # We need to use the ip address that shows up in ipconfig for the usb ethernet adapter
        # that handles the comunication between the PC and the brick
        #print("Setting up left side Server\nAddress: " + left_host + "\nPort: " + str(port))
        print("Setting up right side Server\nAddress: " + right_host + "\nPort: " + str(port))
        
        rightsocket.bind((right_host, port))
        #leftsocket.bind((left_host, port))
        
        # queue up to 5 requests
        rightsocket.listen(5) 
        #leftsocket.listen(5)
        self.r_conn, right_addr = rightsocket.accept()
        #self.l_conn, left_addr = leftsocket.accept()
        #self.conn_dict = {Side.LEFT: self.l_conn, Side.RIGHT: self.r_conn}
        self.conn_dict = {Side.LEFT: 0, Side.RIGHT: self.r_conn}
        self.queue = Queue()
        print(f"Connected to: {right_addr}")
        #print(f"Connected to: {left_addr}")
        
    def send_data(self, flick: float, twist: float, speed: int, type: str, side: str):
        """Sends data to the brick client

        Args:
            flick (float): flick angle
            twist (float): twist angle
            speed (int): motor target speed
            type (MessageType): type of message
            side (Side): side to send the message to
        """
        data = brick_message(flick, twist, speed, type)
        print(f"Sending Data: ({data}) to brick.")
        self.conn_dict[side].send(data)
        # Waiting for the client (ev3 brick) to let the server know that it is done moving
        self.conn_dict[side].recv(128).decode("UTF-8")
        # self.queue.put(reply)

    def send_termination(self):
        """Sends a termination message to the client. This will cause the client 
           to exit "cleanly", after stopping the motors."""
        self.r_conn.send("EXIT".encode("UTF-8"))
        self.l_conn.send("EXIT".encode("UTF-8"))