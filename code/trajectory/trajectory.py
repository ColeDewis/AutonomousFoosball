class Trajectory:
    def __init__(self, direction_vector, initial_position):
        self.direction_vector = direction_vector
        self.initial_position = initial_position

    def time_to_goal_line(self):
        # Calculate the time it takes to hit the goal line (y = 63.5)
        t_goal = (63.5 - self.initial_position[1]) / self.direction_vector[1]
        return t_goal

    def time_to_wall_collision(self):
        # Calculate the time it takes to hit the left wall (x < 4 cm)
        t_x_left = (4 - self.initial_position[0]) / self.direction_vector[0]

        # Calculate the time it takes to hit the right wall (x > 48 cm)
        t_x_right = (48 - self.initial_position[0]) / self.direction_vector[0]

        # Check if the collision time is positive, return the positive one and wall_hit information
        if t_x_left > 0:
            return t_x_left, "left"
        elif t_x_right > 0:
            return t_x_right, "right"
        else:
            print("invalid direction vector")
            return None, None

    def check_goal_or_collision(self):
        # Get time to goal line and wall collision
        t_goal = self.time_to_goal_line()
        t_collision, wall_hit = self.time_to_wall_collision()

        if t_collision is None or wall_hit is None:
            return

        # Compare times and decide whether it's a goal or collision
        if t_goal < t_collision:
            # Goal reached, break out and return x value for this t
            x_goal = self.initial_position[0] + self.direction_vector[0] * t_goal
            print("Goal!")
            return x_goal
        else:
            # Collision with wall
            print(f"Collision with {wall_hit} wall!")

            # Assume ideal collision and update position and direction vector
            new_x = 4 if wall_hit == "left" else 48
            new_y = self.initial_position[1] + self.direction_vector[1] * t_collision
            new_direction_vector = (-self.direction_vector[0], self.direction_vector[1])

            # Update state after collision
            self.initial_position = (new_x, new_y)
            self.direction_vector = new_direction_vector

            # Recalculate time to goal and wall collision after collision
            return self.check_goal_or_collision()

# Example usage:
direction_vector = (0.0001, 1)  # Replace with actual direction vector
initial_position = (16, 18.5)  # Replace with actual initial position

trajectory_instance = Trajectory(direction_vector, initial_position)
x_value = trajectory_instance.check_goal_or_collision()
print(f"Final x value: {x_value}")
