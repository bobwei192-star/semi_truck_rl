import numpy as np
from .adp.exploration_noise import exploration_noise

def steering_controller(t, x, y, h, g, params):
    # steering feedback control
    if params['velocity'] <= 0:
        K = params['feedbackGain']
        u = -K @ np.array([y, h, g])
    else:
        dh = h - params['forwardTarget'][2]
        bx = x - params['forwardTarget'][0]
        by = y - params['forwardTarget'][1]
        dy = -bx * np.sin(h) + by * np.cos(h)
        K = params['forwardGain']
        u = -K @ np.array([dy, dh, g])
    
    steeringWheelLimit = np.tan(np.deg2rad(35))  # Limited at 35 deg
    e = exploration_noise(t, params)
    usat = np.clip(u + e, -steeringWheelLimit, steeringWheelLimit)
    
    return usat, e


if __name__ == "__main__":
    # Test the function
    params = {
        'velocity': -1,
        'feedbackGain': np.array([[0.1, 0.2, 0.3]]),
        'forwardGain': np.array([0.1, 0.2, 0.3]),
        'forwardTarget': [20, 2, 0.3],
        'noiseLevel': 100
    }
    
    t = 0
    x, y, h, g = 0, 0, 0, 0
    usat, e = steering_controller(t, x, y, h, g, params)
    print(f"Steering control: usat={usat}, noise={e}")
