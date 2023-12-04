# Vision Information Below

### Usage
import the tracker class and then follow a similar implementation in the `if
__name == "__main__"` section in that tracker.py file.

The `Tracker` object will, upon initialization, start up a new thread for the
video capture loop. inside this loop, it will update the ball trajectory
every frame. All that needs to be done in order to get estimated position,
direction, and speed of the ball in world units is to access the necessary
properties of the Tracker.ball_trajectory object, as shown in the implementation
discussed above.

**Note:** Be ready to handle None returns from the property accesses, as that is
what they return when no position/direction/speed can be returned.
