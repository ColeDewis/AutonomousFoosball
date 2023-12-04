import threading
from utils.side import Side

class SharedData:
    """Data object that the two robots share between each other. This is used to communicate
       any necessary/helpful information between the two robots.
    """
    def __init__(self):
        """Initializes the data and creates a lock that the robots should use whenever they access properties
           of this object.
        """
        self.lock = threading.Lock()
        self.pos_dict = {Side.LEFT: None, Side.RIGHT: None}