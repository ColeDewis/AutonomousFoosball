import math, time
import threading
from robot import Robot
from brick_server import BrickServer
from arduino_server import ArduinoServer
from vision.tracker import Tracker
from shared_data import SharedData
from messages.message_type import MessageType
from utils.side import Side

def run_robot(robot: Robot):
    """Function used to run the robot in a thread.

    Args:
        robot (Robot): robot object to run continuously.
    """
    last = time.perf_counter()
    while True:
        robot.run()
        if time.perf_counter() - last > 0.1:
            print(f"{robot.side}, {robot.state}")
            last = time.perf_counter()

if __name__ == "__main__":
    right_brick_host = "192.168.137.1"
    left_brick_host = "169.254.24.231"
    port = 9999
    brick_server = BrickServer(left_brick_host, right_brick_host, port)
    arduino_server = ArduinoServer("COM6", 115200)
    tracker = Tracker(0, img_scale=2)
    shared_data = SharedData()
    
    right_robot = Robot(brick_server, arduino_server, tracker, shared_data, Side.RIGHT)
    left_robot = Robot(brick_server, arduino_server, tracker, shared_data, Side.LEFT)
    
    # TODO: evaluate thread-safety of brickserver/arduinoserver/tracker
    right_robot_thread = threading.Thread(target=run_robot, args=(right_robot,))
    left_robot_thread = threading.Thread(target=run_robot, args=(left_robot,))
    
    # Keyboard input is used to start the game
    input()
    right_robot_thread.start()
    left_robot_thread.start()
    