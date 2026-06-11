import numpy as np
from scipy.linalg import solve_continuous_are
from .abstractSteeringController import AbstractSteeringController
from ..utils.compute_tracking_err_at_target import compute_tracking_err_at_target

class LQRSteeringControl(AbstractSteeringController):
    def __init__(self, vehicle):
        super().__init__()
        self.A = np.zeros((3, 3))
        self.B = np.zeros((3, 1))
        self.Q = np.array([[1, 0, 0],
                          [0, 1, 0],
                          [0, 0, 1]])
        self.R = 100
        self.K = np.array([[0, 0, 0]])
        self.err = np.zeros(3)
        self.vehicle = vehicle
        
        self.A, self.B = vehicle.get_forward_AB()
    
    def reset(self):
        # Solve continuous-time algebraic Riccati equation
        P = solve_continuous_are(self.A, self.B, self.Q, self.R)
        self.K = np.linalg.inv(self.R) @ self.B.T @ P
    
    def compute(self, waypoint):
        xyhg = self.vehicle.get_state()
        dxyh = compute_tracking_err_at_target(xyhg[:3], waypoint[:3])
        dg = xyhg[3] - waypoint[3]
        self.err = np.array([dxyh[0], dxyh[1], dxyh[2], dg])
        self.controlSig = -self.K @ self.err[1:4]
        return self.getSteeringCommand()


if __name__ == "__main__":
    # Test the class
    from vehicle.truckTrailerOnAxle import TruckTrailerOnAxle
    
    truck = TruckTrailerOnAxle()
    controller = LQRSteeringControl(truck)
    controller.reset()
    
    waypoint = np.array([1, 1, 0.1, 0])
    steering = controller.compute(waypoint)
    print(f"Steering command: {steering} deg")
