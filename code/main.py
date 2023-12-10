import time
import threading
from robot import Robot
from brick_server import BrickServer
from arduino_server import ArduinoServer
from vision.tracker import Tracker
from trajectory.trajectory import Trajectory
from utils.shared_data import SharedData
from utils.side import Side

def run_robot(robot: Robot):
    """Function used to run the robot in a thread.

    Args:
        robot (Robot): robot object to run continuously.
    """
    while True:
        robot.run()
        
        # the sleep is needed so that the thread won't hog all the resources,
        # otherwise the camera processing is very slow.
        time.sleep(0.1)

if __name__ == "__main__":
    right_brick_host = "192.168.137.1"
    left_brick_host = "169.254.24.231"
    port = 9999
    brick_server = BrickServer(left_brick_host, right_brick_host, port)
    arduino_server = ArduinoServer("COM6", 115200)
    tracker = Tracker(0, img_scale=2)
    trajectory_planner = Trajectory()
    shared_data = SharedData()
    
    right_robot = Robot(brick_server, arduino_server, tracker, trajectory_planner, shared_data, Side.RIGHT)
    left_robot = Robot(brick_server, arduino_server, tracker, trajectory_planner, shared_data, Side.LEFT)
    
    right_robot_thread = threading.Thread(target=run_robot, args=(right_robot,))
    left_robot_thread = threading.Thread(target=run_robot, args=(left_robot,))
    
    # Keyboard input (press enter) is used to start the game
    input()
    right_robot_thread.start()
    left_robot_thread.start()
    