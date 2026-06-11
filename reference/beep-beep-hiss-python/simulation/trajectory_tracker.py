import numpy as np
from ..planning.taskManager import TaskManager
from ..control.simpleSpeedControl import SimpleSpeedControl
from ..control.lqrSteeringControl import LQRSteeringControl
from ..mapping.truck_trailer_trajectory import TruckTrailerTrajectory
from ..utils.compute_tracking_err_at_target import compute_tracking_err_at_target

class TrajectoryTracker:
    def __init__(self, vehicle):
        self.taskMgnr = TaskManager()
        self.vehicle = vehicle
        self.sampleTime = 0.1
        self.speedController = SimpleSpeedControl(vehicle)
        self.steeringController = LQRSteeringControl(vehicle)
        self.steeringController.reset()
        self.trajectory_ref = None
        self.nearestWP_idx = 1
        self.trajectory_actual = np.zeros((0, 6))
    
    def set_ref_trajectory(self, traj):
        self.trajectory_ref = traj
    
    def track(self):
        for t in np.arange(0, 1000, self.sampleTime):
            self.track_one_step()
            state = self.vehicle.get_state()
            self.trajectory_actual = np.vstack([
                self.trajectory_actual,
                np.array([state[0], state[1], state[2], state[3], 
                         self.vehicle.get_steering_angle(), self.vehicle.v_mps])
            ])
    
    def track_one_step(self):
        self.compute_nearest_waypoint()
        wp = self.trajectory_ref.Data[self.nearestWP_idx, :]
        d_m = self.get_distance_left()
        
        print(f"State: {self.vehicle.x_m:.2f}, {self.vehicle.y_m:.2f}, {self.vehicle.s_deg:.2f}")
        print(f"Tracking waypoint #{self.nearestWP_idx} of {self.trajectory_ref.numWayPoints}")
        print(f"Distance left {d_m:.2f}")
        
        sw = self.steeringController.compute(wp)
        vm = self.speedController.compute(wp[-1], d_m)
        
        self.vehicle.set_steering(sw)
        self.vehicle.set_speed(vm)
        self.vehicle.drive()
    
    def compute_nearest_waypoint(self):
        dist = self.trajectory_ref.Data[:, :2] - np.array([self.vehicle.x_m, self.vehicle.y_m])
        dist = dist[:, 0]**2 + dist[:, 1]**2
        wpIdx = np.argmin(dist)
        self.nearestWP_idx = wpIdx
    
    def get_distance_left(self):
        xyhg = self.vehicle.get_state()
        wp = self.trajectory_ref.Data[self.nearestWP_idx, :]
        dxyh = compute_tracking_err_at_target(xyhg[:3], wp[:3])
        
        # add to get distance left
        d1 = self.trajectory_ref.cumDistLeft[self.nearestWP_idx]
        d2 = dxyh[0]
        
        if self.taskMgnr.taskType == 1:
            d_m = d1 - d2
        else:
            d_m = d1 + d2
        
        return d_m


if __name__ == "__main__":
    # Test the class
    from ..vehicle.truckTrailerOnAxle import TruckTrailerOnAxle
    
    wp = np.array([
        [1, 0, 0, 0, 0, 1],
        [5, 1, 0, 0, 0, 1]
    ])
    
    tt = TruckTrailerOnAxle()
    tckr = TrajectoryTracker(tt)
    trj = TruckTrailerTrajectory(wp)
    tckr.set_ref_trajectory(trj)
    
    # Track for a few steps
    for _ in range(5):
        tckr.track_one_step()
        print(f"Vehicle position: {tt.x_m:.2f}, {tt.y_m:.2f}")
        print()
