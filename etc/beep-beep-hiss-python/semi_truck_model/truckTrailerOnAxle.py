import numpy as np

class TruckTrailerOnAxle:
    """truck and trailer model where the fifth wheel is at the rear center
    of the truck, and it is also the reference point of the model."""
    
    def __init__(self):
        # Params
        self.maxSteeringDeg = 35  # 车轮最大偏角从25度调到35度
        self.maxArticulationDeg = 90  # 最大铰接角 ±90度（国家标准要求）
        self.truckWheelbase = 3.0
        self.trailerWheelbase = 11.0
        
        # Dynamics
        self.v_mps = 1
        self.gear = None
        self.x_m = 0
        self.y_m = 0
        self.h_rad = 0
        self.g_rad = 0
        self.s_deg = 0  # Steering angle
        self.shifting = 1
        
        # Visualization
        self.truckWidth = 3
        self.truckMargin = 1
        self.trailerWidth = 2.8
        self.trailerMarginFront = 1
        self.trailerMarginBack = 3
        self.tireLen = 1.2
        self.tireWidth = 0.3
    
    def set_state(self, xyhg):
        self.x_m = xyhg[0]
        self.y_m = xyhg[1]
        
        # 归一化航向角到 [-π, π] 范围
        self.h_rad = xyhg[2]
        if self.h_rad > np.pi or self.h_rad < -np.pi:
            self.h_rad = ((self.h_rad + np.pi) % (2 * np.pi)) - np.pi
        
        # 限制铰接角在最大范围内
        self.g_rad = xyhg[3]
        max_g_rad = np.deg2rad(self.maxArticulationDeg)
        self.g_rad = min(self.g_rad, max_g_rad)
        self.g_rad = max(self.g_rad, -max_g_rad)
    
    def set_steering(self, swa_deg):
        swa_deg = min(swa_deg, self.maxSteeringDeg)
        swa_deg = max(swa_deg, -self.maxSteeringDeg)
        self.s_deg = swa_deg
    
    def get_steering_angle(self):
        return self.s_deg
    
    def get_steering_angle_rad(self):
        swa = self.get_steering_angle()
        return np.deg2rad(swa)
    
    def get_velocity(self):
        return self.v_mps
    
    def set_speed(self, v):
        self.v_mps = v
    
    def get_state(self):
        return np.array([self.x_m, self.y_m, self.h_rad, self.g_rad])
    
    def get_forward_AB(self):
        A = np.array([[0, 1, 0],
                      [0, 0, 0],
                      [0, 0, -1/self.trailerWheelbase]])
        B = np.array([[0],
                      [1/self.truckWheelbase],
                      [-1/self.truckWheelbase]])
        return A, B
    
    def get_reverse_AB(self):
        A, B = self.get_forward_AB()
        return -A, -B
    
    def drive(self, dt=0.1):
        swa = self.get_steering_angle_rad()
        d_state = np.array([
            np.cos(self.h_rad),
            np.sin(self.h_rad),
            np.tan(swa) / self.truckWheelbase,
            -np.sin(self.g_rad) / self.trailerWheelbase - np.tan(swa) / self.truckWheelbase
        ])
        
        old_state = self.get_state()
        new_state = old_state + dt * self.v_mps * d_state
        self.set_state(new_state)


if __name__ == "__main__":
    # Test the class
    truck = TruckTrailerOnAxle()
    print(f"Initial state: {truck.get_state()}")
    
    truck.set_steering(10)
    print(f"Steering angle: {truck.get_steering_angle()} deg")
    
    truck.drive(0.1)
    print(f"State after driving: {truck.get_state()}")
    
    A, B = truck.get_forward_AB()
    print(f"A matrix:\n{A}")
    print(f"B matrix:\n{B}")
