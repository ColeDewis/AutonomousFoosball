#working to improve this and clean it

import math
import numpy as np

class Trajectory:
    def __init__(self, x0, y0, goal_range, angle_range=(-45, 45), v=1.0):
        self.x0 = x0
        self.y0 = y0
        self.goal_range = goal_range
        self.angle_range = angle_range
        self.v = v

    def update_position(self, vx, vy, t):
        x = self.x0 + vx * t
        y = self.y0 + vy * t
        return x, y

    def ideal_collision(self, vx, vy, normal):
        # Ideal collision reflection formula
        dot = vx * normal[0] + vy * normal[1]
        vx -= 2 * dot * normal[0]
        vy -= 2 * dot * normal[1]
        return vx, vy

    def find_best_angle(self):
        best_angle = None
        min_time = float('inf')

        for angle in range(self.angle_range[0], self.angle_range[1] + 1):
            rad_angle = math.radians(angle)
            direction_vector = np.array([np.cos(rad_angle), np.sin(rad_angle)])
            
            # Calculate velocities based on direction vector
            velocity = self.v
            vx = velocity * direction_vector[0]
            vy = velocity * direction_vector[1]

            time = 0
            while True:
                x, y = self.update_position(vx, vy, time)

                # Check if the ball hits the sides of the table
                if y < 2 or y > 48:  # Assuming side collisions less than 2cm and more than 48cm
                    normal = np.array([1, 0]) if x < 2 else np.array([-1, 0])
                    vx, vy = self.ideal_collision(vx, vy, normal)
                
                time += 0.1  # Increment time for simulation
                
                # Check if the ball reached the goal (considering a small margin)
                if self.goal_range[0] < x < self.goal_range[1]:
                    if time < min_time:
                        min_time = time
                        best_angle = angle
                    break  # Exit the loop once the ball reaches the goal

        return best_angle, min_time

# Example usage
x0, y0 = 17, 25
goal_range = (15, 63.5)  # Assuming the goal is on the y-axis between coordinates 15 and 63.5

trajectory = Trajectory(x0, y0, goal_range)
best_angle, min_time = trajectory.find_best_angle()

print(f"Best Angle: {best_angle} degrees, Time to Goal: {min_time} seconds")