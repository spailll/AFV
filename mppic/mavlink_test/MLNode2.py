
# from utils import generate_path_from_waypoints
import time
import serial
from threading import Thread, Event
from Mavlink import MAVLink, MAVLink_waypoint_list_message, MAVLink_start_mission_message

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 57600
CALLSIGN = 'KJ5IOJ'

waypoints = [[0, 0], [1, 1], [2, 2], [3, 3], [4, 4], [5, 5], [6, 6], [7, 7], [8, 8], [9, 9], [10, 10], [11, 11], [12, 12], [13, 13]]

def pad_coordinates(coords, desired_length=10, pad_value=0.0):
    return coords + [pad_value] * (desired_length - len(coords))
def get_coordinates():
    coordinates = []
    print("Enter coordinates (x, y). Type 'done' when finished.")
    while True:
        user_input = input("Enter x (or 'done' to finish): ")
        if user_input.lower() == 'done':
            break
        try:
            x = float(user_input)
            y_input = input("Enter y: ")
            y = float(y_input)
            coordinates.append((x, y))
        except ValueError:
            print("Invalid input. Please enter numeric values for coordinates.")
    return coordinates
    
def process_coordinates(coordinates):
    group_size = 10
    x_groups = []
    y_groups = []
    for i in range(0, len(coordinates), group_size):
        group = coordinates[i:i+group_size]
        x_coords = [coord[0] for coord in group]
        y_coords = [coord[1] for coord in group]
        x_groups.append(x_coords)
        y_groups.append(y_coords)
    return x_groups, y_groups
                
# coords = get_coordinates()
coords = waypoints
x_list, y_list = process_coordinates(coords)
print("\nYou have entered the following coordinates:")
print(x_list)
print(y_list)

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
except serial.SerialException as e:
    print(f"Error opening serial port {SERIAL_PORT}: {e}")


mav = MAVLink(ser)

for x_group, y_group in zip(x_list, y_list):
    x_group_padded = pad_coordinates(x_group)
    y_group_padded = pad_coordinates(y_group)
    msg = MAVLink_waypoint_list_message(x_coordinate=x_group_padded, y_coordinate=y_group_padded, callsign=CALLSIGN.encode('ascii'), wp_count=len(x_group), current_wp=0)
    mav.send(msg)

msg = MAVLink_start_mission_message(callsign=CALLSIGN)
mav.send(msg)







