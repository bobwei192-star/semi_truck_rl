import numpy as np

def exploration_noise(t, params):
    n = params['noiseLevel'] * np.sin(1 * t) + \
        params['noiseLevel'] * np.sin(7 * t) + \
        params['noiseLevel'] * np.sin(3 * t)
    return n


if __name__ == "__main__":
    # Test the function
    params = {'noiseLevel': 100}
    t = 0
    n = exploration_noise(t, params)
    print(f"Exploration noise at t={t}: {n}")
