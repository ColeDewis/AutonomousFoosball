import time
import math
from robot import Robot
from brick_server import BrickServer
from arduino_server import ArduinoServer
from kinematics.kinematics import Kinematics
from utils.message_type import MessageType

if __name__ == "__main__":
    brick_host = "192.168.137.1"
    brick_port = 9999
    brick_server = BrickServer(brick_host, brick_port)
    arduino_server = ArduinoServer("COM6", 9600)
    # camera = Camera() or similar - init the camera
    # get point from camera - ball_x, ball_y
    # trajectory_planner = TrajectoryPlanner() or similar - init trajectory planner
    robot = Robot(brick_server, arduino_server)
    # robot.run()
    
    time.sleep(2)
    brick_server.send_data(math.pi, math.pi, math.pi, MessageType.RELATIVE)