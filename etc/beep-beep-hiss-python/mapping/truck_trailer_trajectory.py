import numpy as np

class TruckTrailerTrajectory:
    def __init__(self, trajData):
        # trajData is an n by 6 matrix
        self.Data = trajData
        self.compute_cumDistLeft()
    
    def compute_cumDistLeft(self):
        n = self.Data.shape[0]
        self.cumDistLeft = np.zeros(n)
        for ct in range(n-2, -1, -1):
            self.cumDistLeft[ct] = \
                np.linalg.norm(self.Data[ct+1, :2] - self.Data[ct, :2]) \
                + self.cumDistLeft[ct+1]
        self.numWayPoints = n


if __name__ == "__main__":
    # Test the class
    trajData = np.array([
        [1, 0, 0, 0, 0, 1],
        [5, 1, 0, 0, 0, 1]
    ])
    traj = TruckTrailerTrajectory(trajData)
    print(f"Number of waypoints: {traj.numWayPoints}")
    print(f"Cumulative distance left: {traj.cumDistLeft}")
