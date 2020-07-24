#!/usr/bin/env python
import math
import numpy
from geometry_msgs.msg import Point

def latlon2xy(lat, lon, lat0, lon0):
    x = (lon - lon0) * mdeglon(lat0)
    y = (lat - lat0) * mdeglat(lat0)
    return x, y
    
def xy2latlon(x, y, lat0, lon0):
    lon = x/mdeglon(lat0) + lon0
    lat = y/mdeglat(lat0) + lat0
    return lat, lon

def mdeglon(lat0):
    lat0rad = math.radians(lat0)
    return (111415.13 * math.cos(lat0rad)
            - 94.55 * math.cos(3.0*lat0rad)
            - 0.12 * math.cos(5.0*lat0rad))
    
def mdeglat(lat0):
    lat0rad = math.radians(lat0)
    return (111132.09 - 566.05 * math.cos(2.0*lat0rad)
            + 1.20 * math.cos(4.0*lat0rad)
            - 0.002 * math.cos(6.0*lat0rad))

def heading_correction(origin_x, origin_y, angle, point):

    sin_ang = math.sin(math.radians(angle))
    cos_ang = math.cos(math.radians(angle))

    # Translate point to origin before rotation
    point.x -= origin_x
    point.y -= origin_y

    # Rotate the point
    new_x = point.x * cos_ang - point.y * sin_ang
    new_y = point.x * sin_ang + point.y * cos_ang

    # Reproject point back out
    point.x = new_x + origin_x
    point.y = new_y + origin_y

    return point

def calibrate_util(point_x, point_y, lat_x, lat_y, anchor_lat, anchor_lon):
    """ A calibration utility for finding the optimal heading angle for use.

    Args:
        point_x: Where the corresponding latitude,longitude is but in ROS coordinates
        point_y: Same as point_x
        lat_x: Where the corresponding point coordinates are but in latitude, longitude
        lat_y: Same as lat_x
        anchor_lat: The corresponding latitude, longitude of the origin of the map
        anchor_lon: Same as anchor_lat
    """
    min_err_angle = 360
    min_err = 999999
    
    for angle in numpy.arange(0, 360, 0.1):
        # Get a fresh point for each heading adjustment
        local_point = Point()
        local_x, local_y = latlon2xy(lat_x, lat_y, anchor_lat, anchor_lon)
        local_point.x = local_x
        local_point.y = local_y

        # Calculate the heading using the step angle and point
        corrected_point = heading_correction(0, 0, angle, local_point)

        # Calculate the error this heading gives
        dist_err = math.sqrt((corrected_point.x-point_x)**2 + (corrected_point.y-point_y)**2)
        if dist_err < min_err:
            min_err = dist_err
            min_err_angle = angle
        
    return min_err_angle


    
