import time
import math
from robot import Robot
from brick_server import BrickServer
from arduino_server import ArduinoServer
from vision.tracker import Tracker
from messages.message_type import MessageType
from utils.side import Side

if __name__ == "__main__":
    right_brick_host = "192.168.137.1"
    left_brick_host = "169.254.23.231"
    port = 9999
    brick_server = BrickServer(left_brick_host, right_brick_host, port)
    arduino_server = ArduinoServer("COM6", 115200)
    tracker = Tracker(0)
    
    right_robot = Robot(brick_server, arduino_server, tracker, Side.RIGHT)
    
    # TODO: evaluate thread-safety of brickserver/arduinoserver/tracker
    left_robot = Robot(brick_server, arduino_server, tracker, Side.LEFT)
    while True:
        right_robot.run()
    # time.sleep(2)
    # brick_server.send_data(math.pi, math.pi, math.pi, MessageType.RELATIVE)