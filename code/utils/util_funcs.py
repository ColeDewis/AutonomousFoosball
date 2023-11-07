import sys
import math

def deg2rad(angle: float):
    """Converts degrees to radians

    Args:
        angle (float): angle to convert

    Returns:
        float: angle in radians
    """
    return angle * (180 / math.pi)

def debug_print(*args, **kwargs):
    """Print debug messages to stderr.

    This shows up in the output panel in VS Code.
    """
    print(*args, **kwargs, file=sys.stderr)