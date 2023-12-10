import numpy as np
import numpy.typing as nptyping

# scuffed way of importing from same level package 
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
        # radius of belt wheel ~6.5mm
        circumference = 1.3 * np.pi
        
        # on right, a POSITIVE BELT increases x
        # on left, a POSITIVE BELT decreases x
        if side == Side.RIGHT:
            self.c = circumference / (2 * np.pi)
        elif side == Side.LEFT:
            self.c = -circumference / (2 * np.pi)
        
        # translation distances from the motor across the joint link.
        # Since these can vary slightly, have separate cases
        if side == Side.RIGHT:
            self.d1 = 8.5
            self.d2 = 3
        elif side == Side.LEFT:
            self.d1 = 8.5
            self.d2 = 3
            
        # We have to multiply some entries in our homog. matrix by -1 if we are on the left side
        self.i_const = -1 if side == Side.LEFT else 1
        
        # Translations from the world frame (corner) to the flick motor
        # These also define the base frame of the robot.
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
        
        # we know x = self.tx1 + rotation_x_change + self.c * theta_one. 
        required_belt_theta = (hit_position - rotation_x_change - self.tx1) / self.c 
                
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
        # translates us to center of motor 2 
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
        
        # rotation and translation for 2nd homog. transform.
        # translates us to center of motor 3
        # have to take negative of the angle
        r2 = np.array(
            [
                [1, 0, 0, 0], 
                [0, np.cos(self.i_const * -theta_two), -np.sin(self.i_const * -theta_two), 0], 
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
        # translates us to the end effector
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
        
        # generate full homog. transform and postmultiply by [0; 0; 0; 1] to get [x; y; z; 1]
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
    # Example/test code
    kin = Kinematics(Side.LEFT)
    kin2 = Kinematics(Side.RIGHT)
    print(kin2.forward_kinematics(0, np.pi/3, 0))
    print(kin.inverse_kinematics(25.0, np.pi/4))
    print(kin2.inverse_kinematics(32.0, -np.pi/4))
    print(kin.inverse_kinematics(12.0, 0))