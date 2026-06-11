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
    
    def get_corner_positions(self):
        """获取车辆8个角的位置（世界坐标，单位：米）
        
        编号规则：
        牵引车头4个角（编号1-4）:
        1 - 车头右前角
        2 - 车头右后角
        3 - 车头左后角
        4 - 车头左前角
        
        挂车4个角（编号5-8）:
        5 - 挂车右前角（靠近车头）
        6 - 挂车右后角
        7 - 挂车左后角
        8 - 挂车左前角（靠近车头）
        """
        x, y, h, g = self.x_m, self.y_m, self.h_rad, self.g_rad
        
        # 车头尺寸
        t_len = self.truckWheelbase + 2 * self.truckMargin
        t_wid = self.truckWidth
        
        # 车头4个角的相对位置（相对于车头中心点）
        # 车头中心点在车头几何中心，车头长度方向沿h方向
        t_corners_rel = np.array([
            [t_len/2, -t_wid/2],   # 1: 右前
            [-t_len/2, -t_wid/2],  # 2: 右后
            [-t_len/2, t_wid/2],   # 3: 左后
            [t_len/2, t_wid/2]     # 4: 左前
        ])
        
        # 旋转并转换到世界坐标
        cos_h, sin_h = np.cos(h), np.sin(h)
        rot_matrix = np.array([[cos_h, -sin_h], [sin_h, cos_h]])
        truck_corners = np.dot(t_corners_rel, rot_matrix.T) + np.array([x, y])
        
        # 挂车尺寸
        tr_len = self.trailerWheelbase + self.trailerMarginFront + self.trailerMarginBack
        tr_wid = self.trailerWidth
        
        # 挂车中心点（相对于车头中心点）
        tr_center_rel_x = -t_len/2 + self.truckMargin + (self.trailerMarginFront - tr_len/2) * np.cos(g)
        tr_center_rel_y = (self.trailerMarginFront - tr_len/2) * np.sin(g)
        tr_center_x = x + tr_center_rel_x * np.cos(h) - tr_center_rel_y * np.sin(h)
        tr_center_y = y + tr_center_rel_x * np.sin(h) + tr_center_rel_y * np.cos(h)
        
        # 挂车4个角的相对位置（相对于挂车中心点）
        tr_corners_rel = np.array([
            [tr_len/2, -tr_wid/2],   # 5: 右前（靠近车头）
            [-tr_len/2, -tr_wid/2],  # 6: 右后
            [-tr_len/2, tr_wid/2],   # 7: 左后
            [tr_len/2, tr_wid/2]     # 8: 左前（靠近车头）
        ])
        
        # 旋转并转换到世界坐标
        tr_h = h + g
        cos_trh, sin_trh = np.cos(tr_h), np.sin(tr_h)
        rot_matrix_tr = np.array([[cos_trh, -sin_trh], [sin_trh, cos_trh]])
        trailer_corners = np.dot(tr_corners_rel, rot_matrix_tr.T) + np.array([tr_center_x, tr_center_y])
        
        # 返回8个角的位置
        return np.vstack([truck_corners, trailer_corners])


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
    
    # Test corner positions
    corners = truck.get_corner_positions()
    print("\nCorner positions:")
    for i, (cx, cy) in enumerate(corners):
        print(f"  Corner {i+1}: ({cx:.3f}, {cy:.3f})")
