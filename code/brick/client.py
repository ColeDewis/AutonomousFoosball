#!/usr/bin/python3       
# RUN ON BRICK
    
"""Part of the given Visual servoing code. Unmodified
"""
    
import socket
import sys
from foosball_player import FoosballPlayer

def debug_print(*args, **kwargs):
    """Print debug messages to stderr.

    This shows up in the output panel in VS Code.
    """
    print(*args, **kwargs, file=sys.stderr)

# This class handles the client side of communication. It has a set of predefined messages to send to the server as well as functionality to poll and decode data.
class Client:
    def __init__(self, host, port):
        # We need to use the ipv4 address that shows up in ipconfig in the computer for the USB. Ethernet adapter handling the connection to the EV3
        debug_print("Setting up client\nAddress: " + host + "\nPort: " + str(port))
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        self.s.connect((host, port))                               
        
    # Block until a message from the server is received. When the message is received it will be decoded and returned as a string.
    # Output: UTF-8 decoded string containing the instructions from server.
    def pollData(self):
        debug_print("Waiting for Data")
        data = self.s.recv(128).decode("UTF-8")
        debug_print("Data Received")
        return data
    
    # Sends a message to the server letting it know that the movement of the motors was executed without any inconvenience.
    def sendDone(self):
        self.s.send("DONE".encode("UTF-8"))

    # Sends a message to the server letting it know that there was an isse during the execution of the movement (obstacle avoided) and that the initial jacobian should be recomputed (Visual servoing started from scratch)
    def sendReset(self):
        self.s.send("RESET".encode("UTF-8"))

if __name__ == "__main__":
    host = "192.168.137.1"
    port = 9999
    client = Client(host, port)
    robot_player = FoosballPlayer()
    while True:
        data = client.pollData()
        if data == "EXIT":
            robot_player.reset_to_home()
            break

        ret = robot_player.parse_data(data)
        if ret:
            client.sendDone()
        else:
            client.sendReset()
