#!/usr/bin/python3       
# RUN ON BRICK

"""Part of the given code from eClass, modified for our purposes
"""

import socket
from foosball_player import FoosballPlayer

import sys
sys.path.append("..")
from utils.util_funcs import debug_print

class Client:
    """Class to handle client side of communication.
    """
    def __init__(self, host, port):
        # We need to use the ipv4 address that shows up in ipconfig in the computer for the USB. Ethernet adapter handling the connection to the EV3
        debug_print("Setting up client\nAddress: " + host + "\nPort: " + str(port))
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.s.connect((host, port))                               
        
    def poll_data(self):
        """Block until a message from the server is received. When the message is received it will be decoded and returned as a string.

        Returns:
            str: UTF-8 decoded string containing instructions from server.
        """
        debug_print("Waiting for Data")
        data = self.s.recv(128).decode("UTF-8")
        debug_print("Data Received")
        return data
    
    def send_done(self):
        """Send a message telling ther server motor movement succeeded."""
        self.s.send("DONE".encode("UTF-8"))

    def send_reset(self):
        """Send a message to the server indicating there was an issue during movement."""
        self.s.send("RESET".encode("UTF-8"))

if __name__ == "__main__":
    host = "192.168.137.1"
    host2 = "169.254.24.231"
    port = 9999
    client = Client(host2, port)
    robot_player = FoosballPlayer()
    while True:
        data = client.poll_data()
        if data == "EXIT":
            robot_player.reset_to_home()
            client.send_done()
            break

        ret = robot_player.parse_data(data)
        if ret:
            client.send_done()
        else:
            client.send_reset()
