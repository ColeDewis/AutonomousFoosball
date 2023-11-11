#!/usr/bin/python
from brick.foosball_player import FoosballPlayer
from utils.util_funcs import debug_print

import math

p = FoosballPlayer()
p.move_angles([-2*math.pi, math.pi / 4, -math.pi / 4])
p.reset_to_home()
p.reset_motors()
