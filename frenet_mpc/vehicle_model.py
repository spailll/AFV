import numpy as np

class VehicleModel:
    def __init__(self, wheelbase=2.5):
        self.L = wheelbase

    def update_state(self, state, delta, a, dt):
        x, y, theta, v = state

        x += v * np.cos(theta) * dt
        y += v * np.sin(theta) * dt
        theta += v / self.L * np.tan(delta) * dt
        v += a * dt

        theta = (theta + np.pi) % (2 * np.pi) - np.pi

        return np.array([x, y, theta, v])
