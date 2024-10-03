import numpy as np
from mppic import RMPPIController
from utils import generate_path_from_waypoints

def main():
    # Path given as x and y coordinates
    # path_x = np.linspace(0, 10, 100)
    # path_y = np.sin(path_x / 2)  # Example path (Sine wave)
    waypoints = np.array([
        [0.0, 0.0],
        [50.0, 0.0],
        [50.0, 50.0],
        [100.0, 50.0],
        [100.0, 0.0],
        [150.0, 0.0],
        [150.0, 50.0],
        [200.0, 50.0],
        [200.0, 100.0],
        [250.0, 100.0],
        [250.0, 0.0]
    ])

    path_x, path_y = generate_path_from_waypoints(waypoints, corner_radius=5.0, num_points_per_arc=20)

    print (path_x, path_y)

    # Initialize the RMPPI controller
    controller = RMPPIController(path_x, path_y)

    # Simulate control and follow the path
    controller.simulate_control()

if __name__ == '__main__':
    main()
