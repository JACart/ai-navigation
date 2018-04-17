#!/usr/bin/env python

import sys
import tf
import rospy
import numpy as np

import math

#This class still needs testing

from geometry_msgs.msg import Pose, Point #point is able to be used for path planning in a 3d space
from sensor_msgs.msg import NavSatFix #standard ros message for GPS data

#current anchor is the center of the xlabs building, with direction pointing directly north
anchor_lat = 38.431960
anchor_long = -78.875910
anchor_elev = 396 #meters above sea level
anchor_theta = 0 #angle pointing directly north
earth_radius = 6371000 #meters

M = np.array([[-0.45644886, -0.77822273, -2.79976147],
    [ 0.84299717, -0.50090785, 41.31691467],
    [ 0.0, 0.0, 1.0]])

"""
Always go from GPS to XYZ as soon as you can. Then never go back. Ever. Dont even think about it.
If you get a batch of waypoints in lat/long, immediately convert all of them. Then do whatever other math you wanted to do.
Through a combination of an anchor point, distance_between_points, and direction between points, we can convert any lat/long point to an xyz point in our coordinate frame

TODO: Refactor and rename to clarify between lat/long points (maybe call them coordinates) and xyz points
"""
    
def get_point(current_gps):
    """
    Returns a Point() (xyz) of a lat/long relative to the set anchor point
    
    This relies on the xyz_between_points method
    
    Zack
    """
    relative_point = Point()
    current_lat = current_gps.latitude
    current_long = current_gps.longitude
    
    result = xy_between_points(anchor_lat, anchor_long, anchor_theta, current_lat, current_long)
    
    zack_point = np.array([[result[0]], [result[1]], [1]])
    map_point = np.dot(M, zack_point)
    relative_point.x = map_point[0,0]
    relative_point.y = map_point[1,0]
    return relative_point
    """
    relative_point.x = result[0]
    relative_point.y = result[1]
    return relative_point
    """
    
def xyz_between_points(lat1, lng1, elev1, theta1, lat2, lng2, elev2):
    """
    This method calculates the xyz difference between points in meters
    
    This method relies on distance_between_points, and direction_between_points
    
    Zack
    """
    distance_flat = distance_between_points(lat1,lng1,lat2,lng2)
    angle = math.radians( (theta1+direction_between_points(lat1,lng1,lat2,lng2)) % 360 )
    forward = math.cos(angle)*distance_flat
    sideways = math.sin(angle)*distance_flat
    verticle = elev2-elev1
    return (forward, sideways, verticle)

def distance_between_points(lat1, lng1, lat2, lng2):
    """
    :return: distance (meters) between two points. This is approximate as the math assumes the earth is a sphere

    https://www.movable-type.co.uk/scripts/latlong.html
    uses meters for earth radius
    
    Zack
    """
    global earth_radius
    lat1 = math.radians(lat1)
    lng1 = math.radians(lng1)
    lat2 = math.radians(lat2)
    lng2 = math.radians(lng2)

    a = math.sin((lat2-lat1)/2)*math.sin((lat2-lat1)/2) + math.cos(lat1) * math.cos(lat2) * math.sin((lng2-lng1)/2)*math.sin((lng2-lng1)/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt((1-a)))
    distance = earth_radius * c
    return distance
def direction_between_points(lat1, lng1, lat2, lng2):
    """
    :return: bearing between two points (degrees)
    This calculates for the bearing between two points.
    
    Zack
    """
    lat1 = math.radians(lat1)
    lng1 = math.radians(lng1)
    lat2 = math.radians(lat2)
    lng2 = math.radians(lng2)

    bearing = math.atan2((math.sin(lng2-lng1)*math.cos(lat2)),(math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(lng2-lng1)))
    bearing = (math.degrees(bearing)+360)%360
    return bearing
    
def xy_angle_between_points(point1, point2):
    distx = point2.x - point1.x
    disty = point2.y - point1.y
    #y is forward while x is sideways
    return (math.atan2(distx, disty)*180/math.pi)%360
    
def xyz_dist_between_points(point1, point2):
    """
    distance between two xyz points in a 3-dimensional coordinate plane
    """
    distx = point1.x - point2.x
    disty = point1.y - point2.y
    distz = point1.z - point2.z
    return math.sqrt(disty ** 2 + distx ** 2 + distz ** 2)
    
def xyz_to_lat_long(point):
    """
    :return: lat-long based on a single point in meters (0,0,0 represents xlabs)
    
    Avoid using this at all costs there is really no reason this function should ever be called on Gods green earth. 
    Will delete as soon as I meet in person with team.
    """
    
    """
    global anchor_lat, anchor_long, earth_radius	
    x = point.x
    y = point.y
    lat1 = math.radians(anchor_lat)
    lng1 = math.radians(anchor_long)
    bearing = math.atan2(x, y)
    distance = math.sqrt(x*x + y*y)
    angular_dist = distance/earth_radius
    lat2 = math.asin(math.sin(lat1) * math.cos(angular_dist) + math.cos(lat1) * math.sin (angular_dist) * math.cos(bearing))
    lng2 = lng1 + math.atan2( math.sin(bearing) * math.sin(angular_dist) * math.cos(lat1),
	    math.cos(angular_dist) - math.sin(lat1) * math.sin(lat2))   
    return (lat2, lng2)
    """
    pass

def lat_long_to_xyz(lat, lng):
    """
    Converts any lat/long point to the xyz frame using the anchor point
    
    This method is redundant to the first method in the class
    """
    xyz = xyz_between_points( lat, lng, 0, direction_between_points(lat, lng, anchor_lat, anchor_long), anchor_lat, anchor_long, 0)

    return Point(xyz[0], xyz[1], xyz[2])

def xy_between_points(lat1, lng1, theta1, lat2, lng2):
    distance_flat = distance_between_points(lat1,lng1,lat2,lng2)
    angle = math.radians( (theta1+direction_between_points(lat1,lng1,lat2,lng2)) % 360 )
    sideways = math.cos(angle)*distance_flat
    forward = math.sin(angle)*distance_flat
    return (forward, sideways)

def midpoint(p1, p2):
    """
    used to find a geometric midpoint in an x/y coordinate frame
    """
    x = (p1.x + p2.x) / 2.0
    y = (p1.y + p2.y) / 2.0

    return Point(x, y, 0)

def add_intermediate_points(points, threshold):
    """
    Method for adding more points to a list of geometric points within a threshold.
    """
    #threshold = 5.0

    #keeps the final point to add at the end
    final = points[0]

    newPoints = []
    prev = points.pop()

    #loop until points is empty
    #we are using points as a stack here
    while points:
        #get the next point on the stack
        new = points.pop()
        dist = xyz_dist_between_points(prev, new)

        #if the distance is greater than the threshold
        if dist > threshold:
            #put the point on the stack
            points.append(new)
            mid = midpoint(new, prev)
            #add the midpoint to the stack
            points.append(mid)
        else:
            #add the start point to the newPoints and switch new point to the prev point
            newPoints.append(prev)
            prev = new

    #add the final point
    newPoints.append(final)
    newPoints.reverse()
    return newPoints


