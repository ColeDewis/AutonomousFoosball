import numpy as np
import numpy.typing as nptyping

# scuffed way of importing from same level package since ".." wouldn't work
from pathlib import Path
import sys
_parent_dir = Path(__file__).parent.parent.resolve()
sys.path.insert(0, str(_parent_dir))
from utils.side import Side
sys.path.remove(str(_parent_dir))


class Kinematics():
    def __init__(self, side: Side):
        """Initialize a kinematics object

        Args:
            side (Side): the side that the kinematics is for
        """
        # belt wheel FULL diameter = 1.6cm. so radius 0.8, but take off a mm and a half.
        # (because teeth are within radius) - around 6.5mm is an accurate radius.
        circumference = 1.3 * np.pi
        
        # Tension is REALLY important for this belt calc to be accurate.
        # The constant that converts belt angle to distance - that is,
        # belt distance d = c * theta_one
        
        # on right, a POSITIVE BELT increases x
        # on left, a POSITIVE BELT decreases x
        if side == Side.RIGHT:
            self.c = circumference / (2 * np.pi)
        elif side == Side.LEFT:
            self.c = -circumference / (2 * np.pi)
        
        # translation distances from the motor across the joint link
        if side == Side.RIGHT:
            self.d1 = 8.5
            self.d2 = 3
        elif side == Side.LEFT:
            self.d1 = 8.5
            self.d2 = 3
            
        # On right, a POSITIVE TWIST decreases x and y
        # on left, a POSITIVE TWIST increases x and y
        # on right, a POSITIVE FLICK decreases y
        # on left, a POSITIVE FLICK increases y
        # We have to multiply some entries in our homog. matrix by -1 if we are on the left side
        self.i_const = -1 if side == Side.LEFT else 1
        
        # Translations from the world frame (corner) to the flick motor
        # These also define the base frame of the robot.
        # TODO: update this for both sides to be more accurate
        if side == Side.RIGHT:
            self.tx1, self.ty1, self.tz1 = 5.5, 12, 14.5
        elif side == Side.LEFT:
            self.tx1, self.ty1, self.tz1 = 44, 72, 14.5
        
    def inverse_kinematics(self, hit_position: float, hit_angle: float) -> float:
        """Given a necessary hit position and angle, returns the angles
           the motors need to turn to hit the ball at that angle and position

        Args:
            hit_position (float): position in world coords to hit ball at
            hit_angle (float): angle in world coords to hit ball at (rad)
            
        Returns:
            float: angle to move the belt
        """
        # Using our angles, use tf3 to determine the x distance change
        # from turning to the angle we want. 
        # then we just need to adjust our target position using that value,
        # and do inverse calculations on the belt angle only.
        
        # the hit angle is actually directly the twist angle. 
        # however, we need to account for how this changes the x position
        # at the point of contact and adjust belt theta using this
        # to do this, we use the homogenous transform for only the
        # third transformation. The reason we can use only the third is
        # it is the only one affecting x position when we actually hit the ball.
        # rotation and translation for 3rd homog. transform
        r3 = np.array(
            [
                [np.cos(hit_angle), -np.sin(hit_angle), 0, 0],
                [np.sin(hit_angle), np.cos(hit_angle), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ]
        )
        t3 = np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, self.i_const * self.d2],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ]
        )
        tf3 = np.matmul(r3, t3)
        rotation_x_change = np.matmul(
            tf3,
            np.array(
                [
                    [0],
                    [0],
                    [0],
                    [1],
                ]
            ), 
        )[0][0]
        
        # we know distance = self.tx1 + self.c * theta_one. So, the theta we need
        # is simply the distance we need / self.c
        required_belt_theta = ((hit_position + rotation_x_change) - self.tx1) / self.c 
                
        return required_belt_theta
    
    def forward_kinematics(self, theta_one: float, theta_two: float, theta_three: float) -> nptyping.ArrayLike:
        """Forward Kinematics of the robot: use homog. transforms to output position
           based on angles.
           
        Args:
                theta_one (float): angle of belt motor (rad)
                theta_two (float): angle of flick motor (rad)
                theta_three (float): angle of twist motor (rad)
                
        Returns:
            np.array containing world coordinates of the end effector.
        """
        
        # rotation and translation for our first homog. transform  
        r1 = np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )
        t1 = np.array(
            [
                [1, 0, 0, self.tx1 + self.c * theta_one], 
                [0, 1, 0, self.ty1], 
                [0, 0, 1, self.tz1], 
                [0, 0, 0, 1]
            ]
        )
        tf1 = np.matmul(r1, t1)
        
        # rotation and translation for 2nd homog. transform
        r2 = np.array(
            [
                [1, 0, 0, 0], 
                [0, np.cos(self.i_const * -theta_two), -np.sin(self.i_const * -theta_two), 0], # have to take negative of the angle
                [0, np.sin(self.i_const * -theta_two), np.cos(self.i_const * -theta_two), 0], 
                [0, 0, 0, 1]
            ]
        )
        t2 = np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, -self.d1],
                [0, 0, 0, 1]
            ]
        )
        tf2 = np.matmul(r2, t2)
        
        # rotation and translation for 3rd homog. transform
        r3 = np.array(
            [
                [np.cos(theta_three), -np.sin(theta_three), 0, 0],
                [np.sin(theta_three), np.cos(theta_three), 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ]
        )
        t3 = np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, self.i_const * self.d2],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ]
        )
        tf3 = np.matmul(r3, t3)
        
        # generate full homog. transform and postmultiply by [0; 0; 0; 1]
        tf = np.matmul(np.matmul(tf1, tf2), tf3)
        return np.matmul(
            tf,
            np.array(
                [
                    [0],
                    [0],
                    [0],
                    [1],
                ]
            ), 
        )     
        
if __name__ == "__main__":
    #kin = Kinematics(Side.LEFT)
    kin2 = Kinematics(Side.RIGHT)
    #print(kin.forward_kinematics(0, -np.pi/4, 0))
    print(np.transpose(kin2.forward_kinematics(0, 0, 0))[0][:2])
    #print(kin.inverse_kinematics(25.0, np.pi/4))
    #print(kin2.inverse_kinematics(32.0, -np.pi/4))
    #print(kin.inverse_kinematics(12.0, 0))