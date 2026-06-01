import numpy as np
from scipy.linalg import solve_continuous_are

def get_default_truck_trailer_params():
    params = {}
    params['velocity'] = -1
    params['truckWheelbase'] = 3.0
    params['trailerWheelbase'] = 11.0
    params['feedbackGain'] = local_get_default_k()
    params['forwardGain'] = local_get_fwd_k()
    
    # for control
    params['Q'] = np.array([[1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])
    params['R'] = 100
    params['noiseLevel'] = 100
    params['forwardTarget'] = [20, 2, 0.3]
    
    # for viz
    params['truckWidth'] = 3
    params['truckMargin'] = 1
    params['trailerWidth'] = 2.8
    params['trailerMarginFront'] = 1
    params['trailerMarginBack'] = 3
    params['tireLen'] = 1.2
    params['tireWidth'] = 0.3
    
    return params


def local_get_default_k():
    # offLinePolicyIteration
    D1 = 3.0     # trailer wheelbase
    D2 = 6.0   # tractor wheelbase
    A = np.array([[0, -1, 0],
                  [0, 0, 0],
                  [0, 0, 1/D2]])
    B = np.array([[0],
                  [-1/D1],
                  [1/D1]])
    Q = np.array([[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 1]])
    R = 1000
    
    # Solve continuous-time algebraic Riccati equation
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    
    return K


def local_get_fwd_k():
    # offLinePolicyIteration
    D1 = 3.0     # trailer wheelbase
    D2 = 6.0   # tractor wheelbase
    A = np.array([[0, 1],
                  [0, 0]])
    B = np.array([[0],
                  [1/D1]])
    Q = np.array([[1, 0],
                  [0, 1]])
    R = 1000
    
    # Solve continuous-time algebraic Riccati equation
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    K = np.append(K, 0)  # Add zero for third state
    
    return K


if __name__ == "__main__":
    params = get_default_truck_trailer_params()
    print("Default parameters:")
    for key, value in params.items():
        print(f"{key}: {value}")
