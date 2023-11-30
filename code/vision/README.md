# Vision Information Below

### Usage
import the tracker class and then follow a similar implementation in the `if
__name == "__main__"` section in that tracker.py file.

The `Tracker` object will, upon initialization, start up a new thread for the
video capture loop. inside this loop, it will update the ball trajectory
every frame. All that needs to be done in order to get estimated position,
direction, and speed of the ball in world units is to access the necessary
properties of the Tracker object, as shown in the implementation discussed above.

**Note:** Currently, the world coordinates for position estimate are around 3-4 cm
off in x, and around 1 cm off in y. Rigorous testing of direction estimates and
speed estimates has not been done, so that is something that will need to be done
soon.

**Note:** Be ready to handle None returns from the property accesses, as that is
what they return when no position/direction/speed can be returned.

### Calibration To-Do's
- potentially skim through checkerboard images and get better, more informative
ones, and follow the stackoverflow stuff you saw online for getting better results
- make sure intrinsic matrix is used properly in computation (do I invert or do
I just use the center of projection values like I am right now? i.e. where do
focal lengths come into play)
- make sure extrinsic matrices are all done properly (do tests with various ball
positions that you measure irl)
- figure out how to deal with projective geometry (negative y values when we are
still in the arena (might be happening due to my assumption of everything lying
on the plane, when our player is not))
