from GPSTransformer import GPSCoordinateTransformer

gps_transformer = GPSCoordinateTransformer(36.168378, -97.067495)

# Create a list of waypoints
waypoints = []
waypoints.append([gps_transformer.gps_to_xy(36.168378, -97.067495)])
waypoints.append([gps_transformer.gps_to_xy(36.168859, -97.067625)])

print(waypoints)

print(gps_transformer.xy_to_gps(10, 10))