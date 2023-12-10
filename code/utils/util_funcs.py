import sys
import math

def rad2deg(angle: float):
    """Converts radians to degrees

    Args:
        angle (float): angle to convert

    Returns:
        float: angle in degrees
    """
    return angle * (180 / math.pi)


def debug_print(*args, **kwargs):
    """Print debug messages to stderr.

    This shows up in the output panel in VS Code when used on the EV3 Brick.
    """
    print(*args, **kwargs, file=sys.stderr)
