
import time
import serial
from threading import Thread, Event
from Mavlink import MAVLink, MAVLink_waypoint_list_message, MAVLink_start_mission_message, MAVLink_emergency_stop_message
from utils import generate_path_from_waypoints
import numpy as np
import matplotlib.pyplot as plt

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 57600
CALLSIGN = 'KJ5IOJ'

waypoints = [[0, 0], [0,10], [10,10], [10,20]]

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

path_x, path_y = generate_path_from_waypoints(coords)
real_x, real_y = np.array([0]), np.array([0])

for x_group, y_group in zip(x_list, y_list):
    x_group_padded = pad_coordinates(x_group)
    y_group_padded = pad_coordinates(y_group)
    msg = MAVLink_waypoint_list_message(x_coordinate=x_group_padded, y_coordinate=y_group_padded, callsign=CALLSIGN.encode('ascii'), wp_count=len(x_group), current_wp=0)
    mav.send(msg)

msg = MAVLink_start_mission_message(callsign=CALLSIGN.encode('ascii'))
mav.send(msg)

plt.figure()
plt.ion()
plt.show()
t = 0

try:
    while True:
        plt.cla()
        plt.plot(path_x, path_y, 'r--', label='Target Path' if t == 0 else "")
        plt.plot(real_x, real_y, 'ro', label='Real Path' if t == 0 else "")
        plt.pause(0.01)
        if ser.in_waiting > 0:
            c = ser.read(1)
            msg = mav.parse_char(c)
            if msg is not None:
                msg_type = msg.get_type()
                if msg_type == 'CURRENT_LOCATION':
                    print(msg)
                    real_x.append(msg.x_coordinate)
                    real_y.append(msg.y_coordinate)
            

        t+=1

except KeyboardInterrupt:
    print("Keyboard Interrupt")

finally:
    msg = MAVLink_emergency_stop_message(callsign=CALLSIGN.encode('ascii'))
    mav.send(msg)
    time.sleep(0.1)
 
    ser.close()
    print("Serial port closed.")
 
    plt.ioff()
    plt.show()
    print("Plot closed.")






