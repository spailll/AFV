'''
 Author: Ben Sailor
 Description: This script is designed to convert GPS coordinates to XY coordinates
'''
import math

class GPSCoordinateTransformer:
    def __init__(self, init_lat, init_lon):
        self.init_lat = math.radians(init_lat)
        self.init_lon = math.radians(init_lon)
        print("Initial Latitude: ", self.init_lat)
        print("Initial Longitude: ", self.init_lon)
        self.R = 6371000.0  # Earth radius in meters

    def gps_to_xy(self, lat, lon):
        print("Latitude: ", lat)
        print("Longitude: ", lon)
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)

        delta_lat = lat_rad - self.init_lat
        delta_lon = lon_rad - self.init_lon

        mean_lat = (self.init_lat + lat_rad) / 2.0

        x = self.R * delta_lon * math.cos(mean_lat)
        y = self.R * delta_lat

        return x, y
        
    def xy_to_gps(self, x, y):
        delta_lat = y / self.R
        delta_lon = x / (self.R * math.cos(self.init_lat + delta_lat / 2.0))

        lat_rad = delta_lat + self.init_lat
        lon_rad = delta_lon + self.init_lon

        lat = math.degrees(lat_rad)
        lon = math.degrees(lon_rad)

        return lat, lon