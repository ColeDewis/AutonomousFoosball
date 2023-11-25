#!/usr/bin/python3       
# RUN ON BRICK

"""Part of the given code from eClass, modified for our purposes
"""

import socket

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
        
    def pollData(self):
        """Block until a message from the server is received. When the message is received it will be decoded and returned as a string.

        Returns:
            str: UTF-8 decoded string containing instructions from server.
        """
        debug_print("Waiting for Data")
        data = self.s.recv(128).decode("UTF-8")
        debug_print("Data Received")
        return data
    
    def sendDone(self):
        """Send a message telling ther server motor movement succeeded."""
        self.s.send("DONE".encode("UTF-8"))

    def sendReset(self):
        """Send a message to the server indicating there was an issue during movement."""
        self.s.send("RESET".encode("UTF-8"))

if __name__ == "__main__":
    host = "192.168.137.1"
    host2 = "169.254.200.136"
    host = "172.31.73.195"
    port = 9999
    client = Client(host, port)
    while True:
        data = client.pollData()
        debug_print(data)
