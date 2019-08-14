#!/usr/bin/env python

'''
This is a utility class used when dealing with points given by th gps
'''

import math
import numpy as np
import rospy

from geometry_msgs.msg import Point #point is able to be used for path planning in a 3d space
# from sensor_msgs.msg import NavSatFix #standard ros message for GPS data

#current anchor is the center of the xlabs building, with direction pointing directly north
ANCHOR_LAT = rospy.get_param("/anchor_lat")
ANCHOR_LONG = rospy.get_param("/anchor_long")
ANCHOR_ELEV = rospy.get_param("/anchor_elev") #meters above sea level
ANCHOR_THETA = rospy.get_param("/anchor_theta") #angle pointing directly north
EARTH_RADIUS = 6371000 #meters

"""
This array is used to transform points from the frame based on this anchor point to the map frame
used by sensors.
This array was created using 7 points, their x,y positions in each frame, and their gps coordinates.
The affine matrices method from this code was used:
https://www.lfd.uci.edu/~gohlke/code/transformations.py.html
"""
M = np.array([[-0.45644886, -0.77822273, -2.79976147],
              [0.84299717, -0.50090785, 41.31691467],
              [0.0, 0.0, 1.0]])


"""
Always go from GPS to XYZ as soon as you can. Then never go back. Ever. Dont even think about it
.
If you get a batch of waypoints in lat/long, immediately convert all of them. Then do whatever
other math you wanted to do.
Through a combination of an anchor point, distance_between_points, and direction between any
lat/long point to an points, we can convert xyz point in our coordinate frame

"""
def get_point(curr_gps=None, lat=None, longitude=None):
    """
    Returns a Point() (xyz) of a lat/long relative to the set anchor point

    This relies on the xy_between_coordinates method

    """
    rel_point = Point()
    if curr_gps is not None:
        curr_lat = curr_gps.latitude
        curr_long = curr_gps.longitude
    else:
        curr_lat = lat
        curr_long = longitude

    #The point is found in xyz in reference to the anchor being 0,0,0
    result = xy_between_coordinates(ANCHOR_LAT, ANCHOR_LONG, ANCHOR_THETA, curr_lat, curr_long)

    plan_point = np.array([[result[0]], [result[1]], [1]]) #3rd value is to make point homogenous
    sensors_point = np.dot(M, plan_point) #dot product to get transformed point
    rel_point.x = sensors_point[0, 0]
    rel_point.y = sensors_point[1, 0]
    return rel_point

def xy_between_coordinates(lat_1, lng_1, theta, lat_2, lng_2):
    """
    Returns a tuple of x and y distances between two coordinates

    this relies on distance & direction between coordinates methods

    """
    dist_flat = distance_between_coordinates(lat_1, lng_1, lat_2, lng_2)
    angle = math.radians((theta + direction_between_coordinates(lat_1, lng_1, lat_2, lng_2)) % 360)
    x = math.sin(angle)*dist_flat
    y = math.cos(angle)*dist_flat
    return (x, y)

def distance_between_coordinates(lat_1, lng_1, lat_2, lng_2):
    """
    return: distance (meters) between two points. This is approximate as the math assumes the earth
    is a sphere

    https://www.movable-type.co.uk/scripts/latlong.html
    uses meters for earth radius

    """
    global EARTH_RADIUS
    latt_1 = math.radians(lat_1)
    long_1 = math.radians(lng_1)
    latt_2 = math.radians(lat_2)
    long_2 = math.radians(lng_2)

    a = math.sin((latt_2 - latt_1) / 2)*math.sin((latt_2 - latt_1) / 2) + \
	math.cos(latt_1)*math.cos(latt_2)*math.sin((long_2-long_1) / 2)*math.sin((long_2 - long_1) / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt((1 - a)))
    distance = EARTH_RADIUS*c
    return distance

def direction_between_coordinates(lat_1, lng_1, lat_2, lng_2):
    """
    return: bearing between two points (degrees)
    This calculates for the bearing between two points.

    """
    latt_1 = math.radians(lat_1)
    long_1 = math.radians(lng_1)
    latt_2 = math.radians(lat_2)
    long_2 = math.radians(lng_2)

    bearing = math.atan2((math.sin(long_2 - long_1)*math.cos(latt_2)),
                         (math.cos(latt_1)*math.sin(latt_2) -
                          math.sin(latt_1)*math.cos(latt_2)*math.cos(long_2 - long_1)))
    bearing = (math.degrees(bearing) + 360) % 360
    return bearing
