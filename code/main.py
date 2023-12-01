import math, time
import threading
from robot import Robot
from brick_server import BrickServer
from arduino_server import ArduinoServer
from vision.tracker import Tracker
from messages.message_type import MessageType
from utils.side import Side

def run_robot(robot: Robot):
    """Function used to run the robot in a thread.

    Args:
        robot (Robot): robot object to run continuously.
    """
    while True:
        robot.run()

if __name__ == "__main__":
    right_brick_host = "192.168.137.1"
    left_brick_host = "169.254.24.231"
    port = 9999
    brick_server = BrickServer(left_brick_host, right_brick_host, port)
    arduino_server = ArduinoServer("COM6", 115200)
    tracker = Tracker(0, img_scale=2)
    
    right_robot = Robot(brick_server, arduino_server, tracker, Side.RIGHT)
    left_robot = Robot(brick_server, arduino_server, tracker, Side.LEFT)
    
    # TODO: evaluate thread-safety of brickserver/arduinoserver/tracker
    right_robot_thread = threading.Thread(target=run_robot, args=(right_robot,))
    left_robot_thread = threading.Thread(target=run_robot, args=(left_robot,))
    right_robot_thread.start()
    left_robot_thread.start()
  
    # time.sleep(2)
    # brick_server.send_data(math.pi/4, math.pi/4, 20, MessageType.RELATIVE, Side.LEFT)
    # brick_server.send_data(math.pi/4, math.pi/4, 20, MessageType.RELATIVE, Side.RIGHT)
    