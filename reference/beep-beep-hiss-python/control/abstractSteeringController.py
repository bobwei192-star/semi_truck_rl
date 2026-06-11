import numpy as np
from abc import ABC, abstractmethod

class AbstractSteeringController(ABC):
    def __init__(self):
        self.controlSig = 0
    
    def getSteeringCommand(self):
        return np.rad2deg(self.controlSig)
    
    @abstractmethod
    def compute(self, waypoint):
        pass


if __name__ == "__main__":
    # Test abstract class (cannot instantiate directly)
    print("AbstractSteeringController class defined")
