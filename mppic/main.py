import numpy as np
from mppic import RMPPIController
from utils import generate_path_from_waypoints

def get_user_waypoints():
    """
    Prompts the user to enter waypoints interactively.
    
    :return: A NumPy array of shape (N, 2) containing the waypoints.
    """
    print("Enter waypoints one by one in the format 'x,y'.")
    print("Type 'done' when you are finished.\n")
    
    waypoints = []
    while True:
        user_input = input(f"Enter waypoint {len(waypoints)+1} (x,y) or 'done': ").strip()
        if user_input.lower() == 'done':
            break
        try:
            # Split input by comma or space
            if ',' in user_input:
                x_str, y_str = user_input.split(',')
            else:
                x_str, y_str = user_input.split()
            x = float(x_str)
            y = float(y_str)
            waypoints.append([x, y])
        except ValueError:
            print("Invalid input. Please enter two numerical values separated by a comma or space.")
        except Exception as e:
            print(f"An error occurred: {e}")
    
    if not waypoints:
        print("No waypoints entered. Exiting program.")
        exit(0)
    
    waypoints_array = np.array(waypoints)
    print(f"\nWaypoints entered ({len(waypoints_array)} points):\n{waypoints_array}\n")
    return waypoints_array


def main():
    # Path given as x and y coordinates
    # path_x = np.linspace(0, 10, 100)
    # path_y = np.sin(path_x / 2)  # Example path (Sine wave)
    # waypoints = np.array([
    #    [0.0, 0.0],
    #    [50.0, 0.0],
    #    [50.0, 50.0],
    #    [100.0, 50.0],
    #    [100.0, 0.0],
    #    [150.0, 0.0],
    #    [150.0, 50.0],
    #    [200.0, 50.0],
    #    [200.0, 100.0],
    #    [250.0, 100.0],
    #    [250.0, 0.0]
    #])
    waypoints = get_user_waypoints()

    path_x, path_y = generate_path_from_waypoints(waypoints, corner_radius=5.0, num_points_per_arc=20)

    print (path_x, path_y)

    # Initialize the RMPPI controller
    controller = RMPPIController(path_x, path_y)

    # Simulate control and follow the path
    controller.simulate_control()

if __name__ == '__main__':
    main()
