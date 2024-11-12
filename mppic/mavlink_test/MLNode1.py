# from utils import generate_path_from_waypoints
import time
import serial
from threading import Thread, Event
from Mavlink import MAVLink

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 57600
CALLSIGN = 'YOURCALSGN'

waypoints = []
mission_started = False
        
def handle_waypoint_list(msg):
    # if msg.callsign.decode != CALLSIGN:
    #     print(f"Received waypoints from {msg.callsign}. Ignoring.")
    #     return
    
    wp_count = msg.wp_count
    
    for i in range(wp_count):
        x = msg.x_coordinate[i]
        y = msg.y_coordinate[i]
        
        waypoints.append([x, y])

    print(f"Received {wp_count} waypoints from {msg.callsign}.")
    for idx, wp in enumerate(waypoints):
        print(f"Waypoint {idx+1}: X={wp[0]}, Y={wp[1]}")

def handle_start_mission(msg):
    if msg.callsign != CALLSIGN:
        print(f"Received mission start from {msg.callsign}. Ignoring.")
        return

    mission_started = True
    print(f"Received mission start from {msg.callsign}. Starting mission.")


could_not_open = False
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
    print(f"Opened serial port {SERIAL_PORT} at {BAUD_RATE} baud.")
except Exception as e:
    print(f"Error opening serial port {SERIAL_PORT}: {e}")
    could_not_open = True

if not could_not_open:
    mav = MAVLink(ser)

    while True:
        if ser.in_waiting > 0:
            c = ser.read(1)
            msg = mav.parse_char(c)
            if msg is not None:
                msg_type = msg.get_type()
                if msg_type == 'WAYPOINT_LIST':
                    handle_waypoint_list(msg)
                elif msg_type == 'START_MISSION':
                    handle_start_mission(msg)
            if mission_started == True:
                break

    # path_x, path_y = generate_path_from_waypoints(waypoints, corner_radius=5.0, num_points_per_arc=20)

    print("\nGenerated path:")
    print(waypoints)

    ser.close()