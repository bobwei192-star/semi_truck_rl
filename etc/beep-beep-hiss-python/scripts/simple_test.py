import numpy as np
import sys
import os

# Add parent directory to path to import modules
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from vehicle.truckTrailerOnAxle import TruckTrailerOnAxle
from simulation.trajectory_tracker import TrajectoryTracker
from mapping.truck_trailer_trajectory import TruckTrailerTrajectory

def main():
    # Define waypoints
    wp = np.array([
        [1, 0, 0, 0, 0, 1],
        [5, 1, 0, 0, 0, 1]
    ])
    
    # Create truck, tracker, and trajectory
    tt = TruckTrailerOnAxle()
    tckr = TrajectoryTracker(tt)
    trj = TruckTrailerTrajectory(wp)
    tckr.set_ref_trajectory(trj)
    
    print("Starting trajectory tracking test...")
    print(f"Initial state: {tt.get_state()}")
    print()
    
    # Track for a few steps
    for i in range(10):
        print(f"Step {i+1}:")
        tckr.track_one_step()
        print(f"Vehicle position: {tt.x_m:.2f}, {tt.y_m:.2f}")
        print()

if __name__ == "__main__":
    main()
