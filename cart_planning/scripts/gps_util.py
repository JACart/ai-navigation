#!/usr/bin/env python

import sys
import tf
import rospy
import numpy as np
import affine
import math
import transformations

#This class still needs testing

from geometry_msgs.msg import Pose, Point #point is able to be used for path planning in a 3d space
from sensor_msgs.msg import NavSatFix #standard ros message for GPS data

#current anchor is the center of the xlabs building, with direction pointing directly north
anchor_lat = 38.432150 # 38.431691 # 38.432147 # 38.431960
anchor_long = -78.876106 # -78.876036 #-78.876115 # -78.875910
anchor_elev = 396 #meters above sea level
anchor_theta = 40 #angle pointing directly north, originally: 0
earth_radius = 6371000 #meters

"""
#This array is used to transform points from the frame based on this anchor point to the map frame used by sensors
#This array was created using 7 points, their x,y positions in each frame, and their gps coordinates. 
#The affine matrices method from this code was used: https://www.lfd.uci.edu/~gohlke/code/transformations.py.html
"""


'''M = np.array([[-0.45644886, -0.77822273, -2.79976147],
    [ 0.84299717, -0.50090785, 41.31691467],
    [ 0.0, 0.0, 1.0]])'''
    

v0 = None
v1 = None
M = None


"""
Always go from GPS to XYZ as soon as you can. Then never go back. Ever. Dont even think about it
.
If you get a batch of waypoints in lat/long, immediately convert all of them. Then do whatever other math you wanted to do.
Through a combination of an anchor point, distance_between_points, and direction between points, we can convert any lat/long point to an xyz point in our coordinate frame

TODO: move all the geometry methods to another utility file
"""
    
def get_point(current_gps):
    """
    Returns a Point() (xyz) of a lat/long relative to the set anchor point
    
    This relies on the xy_between_coordinates method
    
    Zack
    """
    relative_point = Point()
    current_lat = current_gps.latitude
    current_long = current_gps.longitude
    
    #The point is found in xyz in reference to the anchor being 0,0,0
    result = xy_between_coordinates(anchor_lat, anchor_long, anchor_theta, current_lat, current_long)
    
    planning_point = np.array([[result[0]], [result[1]], [1]]) #3rd value is to make point homogenous
    sensors_map_point = np.dot(M, planning_point) #dot product to get transformed point
    relative_point.x = sensors_map_point[0,0]
    relative_point.y = sensors_map_point[1,0]
    rospy.loginfo(relative_point)
    return relative_point

def xy_between_coordinates(lat1, lng1, theta1, lat2, lng2):
    """
    Returns a tuple of x and y distances between two coordinates
    
    this relies on distance & direction between coordinates methods
    
    Zack
    """
    distance_flat = distance_between_coordinates(lat1,lng1,lat2,lng2)
    angle = math.radians( (theta1+direction_between_coordinates(lat1,lng1,lat2,lng2)) % 360 )
    x = math.sin(angle)*distance_flat
    y = math.cos(angle)*distance_flat
    return (x, y)

def distance_between_coordinates(lat1, lng1, lat2, lng2):
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
def direction_between_coordinates(lat1, lng1, lat2, lng2):
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
    
    
"""
TODO
Every method below here is a geometry util that should go in a separate file. 
These are used for Pure Pursuit"""
    
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

#X
v0 = [[35.9750671387,
14.0169754028,
-3.60410761833,
17.4223442078,
30.7783546448,
59.0794563293,
47.9315681458],
#Y
[36.4229431152,
1.30921435356,
8.7294960022,
46.9932327271,
43.7685050964,
72.0065460205,
86.720085144]] 





#Latitude
v1 = [[38.431686,
38.432053,
38.432108,
38.431724,
38.431672,
38.431312,
38.431282],
#Longitude
[-78.876137,
-78.876221,
-78.876016,
-78.875895,
-78.876047,
-78.876070,
-78.875866]]


v1_np = np.array(v1)
new_arr = [[],[]]
'''
# Setup for averaging
anchor_lat = 0
anchor_long = 0

for i in range(v1_np.shape[1]):
    
    lat = v1_np[0][i]
    anchor_lat += lat
    
    lng = v1_np[1][i]
    anchor_long += lng
    
# Average for anchor point
anchor_lat = anchor_lat / 7
anchor_long = anchor_long / 7'''

# End averaging, begin converting

for i in range(v1_np.shape[1]):
    
    lat = v1_np[0][i]
    
    lng = v1_np[1][i]

    elem = xy_between_coordinates(anchor_lat, anchor_long, anchor_theta, lat, lng)
    new_arr[0].append(elem[0])
    new_arr[1].append(elem[1])



M = transformations.affine_matrix_from_points(v0, new_arr)
#M = transformations.affine_matrix_from_points(v0, v1_np)
