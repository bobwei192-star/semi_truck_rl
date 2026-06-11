import numpy as np

def compute_tracking_err_at_target(xyh, xyh1):
    dh = xyh[2] - xyh1[2]
    bx = xyh[0] - xyh1[0]
    by = xyh[1] - xyh1[1]
    dx = bx * np.cos(xyh[2]) + by * np.sin(xyh[2])
    dy = -bx * np.sin(xyh[2]) + by * np.cos(xyh[2])
    dxyh = np.array([dx, dy, dh])
    return dxyh


if __name__ == "__main__":
    # Test the function
    xyh = np.array([0, 0, 0])
    xyh1 = np.array([1, 1, 0.1])
    dxyh = compute_tracking_err_at_target(xyh, xyh1)
    print(f"Tracking error: {dxyh}")
