import time
import math
from queue import Queue
from brick_server import BrickServer
from kinematics.kinematics import Kinematics
from utils.message_type import MessageType

if __name__ == "__main__":
    host = "192.168.137.1"
    port = 9999
    server = BrickServer(host, port)
    queue = Queue()

    # camera = Camera() or similar - init the camera
    # get point from camera - ball_x, ball_y
    
    # trajectory_planner = TrajectoryPlanner() or similar - init the trajectory planner
    # hit_x, hit_angle = trajectory_planner.get_required_hit_location() or similar
    
    kin = Kinematics()
    # belt_angle = kin.inverse_kinematics(hit_x, hit_angle)
    # server.send_data(belt_angle, math.pi / 4, hit_angle, MessageType.RELATIVE, queue)
    # server.send_data(0, -math.pi / 4, 0, MessageType.RELATIVE, queue)
    
    time.sleep(2)
    server.send_data(math.pi, math.pi, math.pi, MessageType.RELATIVE, queue)