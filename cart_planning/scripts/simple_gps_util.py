#!/usr/bin/env python
import math
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

    
