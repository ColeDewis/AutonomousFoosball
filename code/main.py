import time
import math
from queue import Queue
from brick_server import BrickServer

if __name__ == "__main__":
    host = "192.168.0.2"
    port = 9999
    server = BrickServer(host, port)
    queue = Queue()

    while True:
        time.sleep(10)
        server.send_angles(math.pi, math.pi, math.pi, queue)