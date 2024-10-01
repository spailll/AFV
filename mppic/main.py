import numpy as np
from mppic import RMPPIController
from vehicle import Vehicle

def main():
    # vehicle = Vehicle()
    # Path given as x and y coordinates
    path_x = np.linspace(0, 10, 100)
    path_y = np.sin(path_x / 2)  # Example path (Sine wave)

    # Initialize the RMPPI controller
    controller = RMPPIController(path_x, path_y)

    # Simulate control and follow the path
    controller.simulate_control()

if __name__ == '__main__':
    main()