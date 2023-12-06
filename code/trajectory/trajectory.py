import math

class Trajectory:
    """
    A class representing the trajectory of an object in a 2D space.

    Attributes:
        left_boundary_value (int): The left boundary value.
        right_boundary_value (int): The right boundary value.
        goal_x_range (tuple): The range of x-values for the goal area.
        goal_line_dict (dict): Dictionary containing goal line positions for each side.

    Methods:
        __init__(): Initializes the Trajectory class.
        time_to_goal_line(initial_position, direction_vector, side) -> float:
            Calculates the time to reach the goal line.
        time_to_wall_collision(initial_position, direction_vector) -> tuple:
            Calculates the time to collision with the left or right boundary.
        position_on_goal_line(initial_position, direction_vector, mode, side) -> tuple:
            Calculates the position on the goal line.
        find_best_angle(opposite_player_x, initial_position, mode, side, direction_vector) -> int:
            Finds the best angle for the trajectory.
    """
    
    # Define constants
    left_boundary_value = 4
    right_boundary_value = 48
    goal_x_range = (15, 35)

    def __init__(self) -> None:
        self.goal_line_dict = {"LEFT": 63.5, "RIGHT": 17}

    def time_to_goal_line(self, initial_position: tuple, direction_vector: tuple, side: str) -> float:
        """
        Calculate the time to reach the goal line.

        Args:
            initial_position (tuple): Initial position (x, y).
            direction_vector (tuple): Direction vector (dx, dy).
            side (str): Side of the goal line ("LEFT" or "RIGHT").

        Returns:
            float: Time to reach the goal line.
        """
        goal_line = self.goal_line_dict[side]
        t_goal = (goal_line - initial_position[1]) / direction_vector[1]
        return t_goal

    def time_to_wall_collision(self, initial_position: tuple, direction_vector: tuple) -> tuple:
        """
        Calculate the time to collision with the left or right boundary.

        Args:
            initial_position (tuple): Initial position (x, y).
            direction_vector (tuple): Direction vector (dx, dy).

        Returns:
            tuple: Time to collision and the side of collision ("left" or "right").
        """
        if direction_vector[0] == 0:
            return float('inf'), float(0)

        t_x_left = (self.left_boundary_value - initial_position[0]) / direction_vector[0]
        t_x_right = (self.right_boundary_value - initial_position[0]) / direction_vector[0]

        if t_x_left > 0:
            return t_x_left, "left"
        elif t_x_right > 0:
            return t_x_right, "right"
        else:
            print("Invalid direction vector")
            return None, None

    def position_on_goal_line(self, initial_position: tuple, direction_vector: tuple, mode: str, side: str) -> tuple:
        """
        Calculate the position on the goal line.

        Args:
            initial_position (tuple): Initial position (x, y).
            direction_vector (tuple): Direction vector (dx, dy).
            mode (str): Trajectory mode ("TOGETHER" or "AGAINST").
            side (str): Side of the goal line ("LEFT" or "RIGHT").

        Returns:
            tuple: Position on the goal line (x, y).
        """
        t_goal = self.time_to_goal_line(initial_position, direction_vector, side)
        t_collision, wall_hit = self.time_to_wall_collision(initial_position, direction_vector)

        if t_collision is None or wall_hit is None:
            return None, None

        if t_goal < t_collision:
            x_goal = initial_position[0] + direction_vector[0] * t_goal
            y_goal = self.goal_line_dict[side]

            if self.goal_x_range[0] <= x_goal <= self.goal_x_range[1]:
                return x_goal, y_goal
            else:
                return x_goal, y_goal
        else:
            new_x = self.left_boundary_value if wall_hit == "left" else self.right_boundary_value
            new_y = initial_position[1] + direction_vector[1] * t_collision
            new_direction_vector = (-direction_vector[0], direction_vector[1])

            new_initial_position = (new_x, new_y)
            new_direction_vector = new_direction_vector

            return self.position_on_goal_line(new_initial_position, new_direction_vector, mode, side)

    def find_best_angle(self, opposite_player_x: float, initial_position: tuple, mode: str, side: str, direction_vector: tuple = (0, 1)) -> int:
        """
        Find the best angle for the trajectory.

        Args:
            opposite_player_x (float): X-coordinate of the opposing player.
            initial_position (tuple): Initial position (x, y).
            mode (str): Trajectory mode ("TOGETHER" or "AGAINST").
            side (str): Side of the goal line ("LEFT" or "RIGHT").
            direction_vector (tuple): Direction vector (dx, dy).

        Returns:
            int: Best angle for the trajectory.
        """
        min_distance = float('inf')  # For 'together' mode
        max_distance = float('-inf')  # For 'against' mode
        best_angle = None

        for angle in range(-45, 46):  # Adjusted the range to include -45 to 45 degrees
            theta = math.radians(angle)
            new_direction_vector = (
                math.sin(theta),
                math.cos(theta) if side == 'LEFT' else -math.cos(theta)
            )

            x_hit, _ = self.position_on_goal_line(initial_position, new_direction_vector, mode, side)

            distance = abs(opposite_player_x - x_hit)

            if mode == 'TOGETHER' and distance < min_distance:
                min_distance = distance
                best_angle = angle
            elif mode == 'AGAINST' and distance > max_distance:
                max_distance = distance
                best_angle = angle

        return best_angle


if __name__ == "__main__":
    # Example usage:
    trajectory_instance = Trajectory()
    opposite_player_x = 30
    initial_position = (10, 20)
    mode = 'TOGETHER'
    side = 'LEFT'
    best_angle = trajectory_instance.find_best_angle(opposite_player_x, initial_position, mode, side)
    print(f"The best angle for the trajectory is: {best_angle} degrees.")



    # Example usage:
    direction_vector = (0.0001, 1)  # Replace with actual direction vector
    initial_position = (16, 18.5)  # Replace with actual initial position
    side = 'RIGHT'
    mode = 'TOGETHER'  # Replace with the desired mode

    # Create an instance of the Trajectory class
    trajectory_instance = Trajectory()

    # Call the check_goal_or_collision method
    x_value, y_value = trajectory_instance.position_on_goal_line(initial_position, direction_vector, mode, side)

    print(f"Final x value : {x_value}, Final y value: {y_value}") 
