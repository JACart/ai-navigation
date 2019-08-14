#!/usr/bin/env python

''' Utility class for simple geometry math '''


import math


from geometry_msgs.msg import Point

def xy_angle_between_points(point_1, point_2):
    """
    gets the angle between two points by creating a vector based
    on the distances
    """
    dist_x = point_2.x - point_1.x
    dist_y = point_2.y - point_1.y

    #y is forward while x is sideways
    return (math.atan2(dist_x, dist_y) * 180 / math.pi) % 360

def xyz_dist_between_points(point_1, point_2):
    """
    distance between two xyz points in a 3-dimensional coordinate plane
    """
    dist_x = point_1.x - point_2.x
    dist_y = point_1.y - point_2.y
    dist_z = point_1.z - point_2.z
    return math.sqrt(dist_y**2 + dist_x**2 + dist_z**2)

def midpoint(p_1, p_2):
    """
    used to find a geometric midpoint in an x/y coordinate frame
    """
    x = (p_1.x + p_2.x) / 2.0
    y = (p_1.y + p_2.y) / 2.0

    return Point(x, y, 0)

def add_intermediate_points(points, threshold):
    """
    Method for adding more points to a list of geometric points within a threshold.
    """

    #keeps the final point to add at the end
    final = points[0]

    new_points = []
    prev = points.pop()

    #loop until points is empty
    while points:

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
            #add the start point to the newPoints and make the set previous point equal to new point
            new_points.append(prev)
            prev = new

    #add the final point
    new_points.append(final)
    new_points.reverse()
    return new_points

