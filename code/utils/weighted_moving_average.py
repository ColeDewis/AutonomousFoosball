import numpy as np
import numpy.typing as nptyping
from collections import deque

if __name__ == "__main__":
    from decay_functions import WeightFunction, ExponentialDecay, TrustDecay
else:
    from utils.decay_functions import WeightFunction, ExponentialDecay, TrustDecay

class Entry:
    """Class for an entry in a WeightedMovingAverage."""
    def __init__(self, value: list):
        """Initialize the entry

        Args:
            value (np.array[float]): value of the entry. Should be a list of any n elements.
        """
        self.value = value
        self.age = 0
    
    def __mul__(self, other) -> float:
        """Overloads the multiplication value for an entry.
        
        We ONLY ever multiply an Entry by its weight, so other
        is ALWAYS a weight

        Args:
            other (float): the weight to multiply by

        Returns:
            float: the entry value times the given weight
        """
        return np.array(self.value) * other

class WeightedMovingAverage:
    """Class to represent a weighted moving average, used for averaging
       trajectories and object estimates."""
    def __init__(self, decay_class: WeightFunction, n: int, max_size: int = np.inf):
        """Initialize a weighted moving average with the
        given decay function.
        
        The entries given to this must be a list of n elements.

        Args:
            decay_func (function): function that describes the decay a point's weight.
                                   should take in one parameter - the point age.
            n (int): number of elements in each entry. 
            max_size (int): max amount of entries allowed. Defaults to infinity.
        """
        self.decay_func = decay_class().weight # the weight function of the passed decay class.
        self.n = n
        self.max_size = max_size
        self.points = deque()
        
    def reset(self):
        """Resets by removing all entries."""
        self.points = deque()
        
    def add_point(self, point_value: list):
        """Add a point to the average. It will initially have
           age of 0.

        Args:
            point_value (np.array[float]): value of the point.

        Raises:
            ValueError: if the given value's size does not match the initial n
        """
        if len(point_value) != self.n:
            raise ValueError(f"Given value does not match required size: {self.n}")
        
        self.points.append(Entry(point_value))
        
        if len(self.points) > self.max_size:
            self.points.popleft()
        
    def age_points(self):
        """Increase the age of all points in the weighted moving average."""
        for point in self.points:
            point.age += 1
    
    def prune(self, age_threshold: int):
        """Prunes the points, removing any at or above the given age_threshold

        Args:
            age_threshold (int): the threshold at which we consider a point to old
                                and want to remove it.
        """
        self.points = [x for x in self.points if x.age < age_threshold]
            
    def output(self) -> nptyping.ArrayLike | None:
        """Output the current value of the weighted moving average.

        Returns:
            None: None if there are no points in the wma.
            list: the current weighted average if any points exist.
        """
        if not self.points:
            return None

        weights = [self.decay_func(point.age) for point in self.points]
        weights = weights / np.sum(weights)
        
        return np.sum(self.points * weights, axis=0)

if __name__ == "__main__":
    wma = WeightedMovingAverage(ExponentialDecay, 2, 10)
    wma.add_point(np.array([1.0, 1.0]))
    wma.age_points()
    wma.add_point(np.array([2.0, 2.0]))
    
    wma2 = WeightedMovingAverage(TrustDecay, 2, 10)
    wma2.add_point(np.array([1.0, 1.0]))
    wma2.age_points()
    wma2.add_point(np.array([2.0, 2.0]))

    wma2.prune(2)
    wma2.age_points()

    print(wma.output())
    print(wma2.output())
    print(wma2.output())
    wma2.prune(2)
    print(wma2.output())
    