import math

class GPSCoordinateTransformer:
    def __init__(self, init_lat, init_lon, init_heading=0.0):
        self.init_lat = init_lat
        self.init_lon = init_lon
        self.heading = math.radians(init_heading) # Heading from true north
        self.R = 6371000.0  # Earth radius in meters

    def gps_to_xy(self, lat, lon, heading):
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)
        heading_rad = math.radians(heading)

        delta_lat = lat_rad - self.init_lat
        delta_lon = lon_rad - self.init_lon

        mean_lat = (self.init_lat + lat_rad) / 2.0

        x = self.R * delta_lon * math.cos(mean_lat)
        y = self.R * delta_lat

        rotation_angle = current_heading - self.init_heading

        cos_angle = math.cos(rotation_angle)
        sin_angle = math.sin(rotation_angle)
        x_rot = x * cos_angle + y * sin_angle
        y_rot = -x * sin_angle + y * cos_angle

        return x_rot, y_rot
        