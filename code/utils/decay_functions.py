import numpy as np
from abc import ABC, abstractmethod

class WeightFunction(ABC):
    """Base class for a decay function to use for a WeightedMovingAverage."""
    @abstractmethod
    def weight(self, age: int):
        """Function that defines the weight given an entries age.

        Args:
            age (int): age of the element
        """
        pass
    
    
class ExponentialDecay(WeightFunction):
    """Class that uses exponential decay as a weight function.

    Args:
        WeightFunction (class): abstract class extended.
    """
    def weight(self, age: int):
        """Uses inverse exponential as weight decay

        Args:
            age (int): age of element
        """
        return 1 / np.exp(age)
    
    
class TrustDecay(WeightFunction):
    """Decay that "loses trust" in every odd input, to be used for kinematics
       where we lose trust in kinematic inputs over time

    Args:
        WeightFunction (class): abstract class extended
    """
    def __init__(self):
        """Initialize the decay."""
        self.call_count = 0
        self.pair_num = 0
        
    def weight(self, age: int):
        """Initially assumes equal confidence, lowering odd inputs confidence
           over time.

        Args:
            age (int): unused
        """
        
        if self.call_count % 2 == 0: # even: trust increase (camera)
            weight = 0.25 + self.pair_num * 0.01 
            value = weight if weight < 0.75 else 0.75
        else: # odd: trust decrease (kinematics)
            weight = 0.75 - self.pair_num * 0.01 
            value = weight if weight > 0.25 else 0.25
            self.pair_num += 1
        
        self.call_count += 1
        return value