import numpy as np
from .steering_controller import steering_controller

def sys_truck_trailer(t, xx, params):
    x = xx[0]
    y = xx[1]
    h = xx[2]
    g = xx[3]
    
    v = params['velocity']
    d1 = params['truckWheelbase']
    d2 = params['trailerWheelbase']
    
    usat, _ = steering_controller(t, x, y, h, g, params)
    
    dx = v * np.cos(h)
    dy = v * np.sin(h)
    dh = v / d1 * usat
    dg = -v / d2 * np.sin(g) - v / d1 * usat
    
    dxx = np.array([dx, dy, dh, dg])
    return dxx


def sys_truck_trailer_wrapper(t, xx, params):
    """Wrapper function for ODE solvers"""
    return sys_truck_trailer(t, xx, params)


if __name__ == "__main__":
    # Test the function
    params = {
        'velocity': -1,
        'truckWheelbase': 3.0,
        'trailerWheelbase': 11.0,
        'feedbackGain': np.array([[0.1, 0.2, 0.3]]),
        'forwardGain': np.array([0.1, 0.2, 0.3]),
        'forwardTarget': [20, 2, 0.3],
        'noiseLevel': 100
    }
    
    xx = np.array([0, 0, 0, 0])
    t = 0
    dxx = sys_truck_trailer(t, xx, params)
    print(f"State derivative: {dxx}")
