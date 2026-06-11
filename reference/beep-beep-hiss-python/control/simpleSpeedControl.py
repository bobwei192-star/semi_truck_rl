class SimpleSpeedControl:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.threshold = 0.1
    
    def compute(self, ref_spd, distance_left):
        spd_cmd = self.feedback(ref_spd)
        if distance_left <= self.threshold:
            spd_cmd = 0
        return spd_cmd
    
    def feedback(self, ref_spd):
        return 0.5 * abs(ref_spd) + 0.5 * abs(self.vehicle.v_mps)


if __name__ == "__main__":
    # Test the class
    from vehicle.truckTrailerOnAxle import TruckTrailerOnAxle
    
    truck = TruckTrailerOnAxle()
    controller = SimpleSpeedControl(truck)
    
    ref_spd = 1.0
    distance_left = 0.5
    spd_cmd = controller.compute(ref_spd, distance_left)
    print(f"Speed command: {spd_cmd}")
