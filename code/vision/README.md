# Vision Information Below

### Usage
import the tracker class and then follow a similar implementation in the `if
__name == "__main__"` section in that tracker.py file.

### Calibration To-Do's
- potentially skim through checkerboard images and get better, more informative
ones, and follow the stackoverflow stuff you saw online for getting better results
- make sure intrinsic matrix is used properly in computation (do I invert or do
I just use the center of projection values like I am right now? i.e. where do
focal lengths come into play)
- make sure extrinsic matrices are all done properly (do tests with various ball
positions that you measure irl)
- figure out how to deal with projective geometry (negative y values when we are still in the arena (might be happening due to my assumption of everything lying
on the plane, when our player is not))
- make sure downscaling images isn't mangling our predictions too much (and if it is, figure out how to deal with scaling it back up)

### Object Detection To-Do's
- get less noisy detections by increasing hsv ranges and dealing with noise, or add in background subtraction stuff
- fix the method of detecting squares by using moments or something
- extract out transform code into its own module to lessen the tasks required
of the tracker module (could make it a class or just a module that stores
all the transforms, pixel scales, and the img_to_world function)
- extract out detection code into its own class or module. The Tracker class
should just be in charge of maintaining location state and dealing with camera
initialization and tracking thread stuff

### Ball-Tracking To-Do's
- build functionality so that the tracker can query this in the get_object_location function and return to user, and add a get_object_trajectory
function to tracker as well. Then the interface for whoever is using it is really
simple. Make it so None is never returned, just return an estimate if no point
was detected (I guess only return None when nothing has been detected yet)
