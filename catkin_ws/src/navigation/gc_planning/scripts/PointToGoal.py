#!/usr/bin/env python
import sys
import tf

from geometry_msgs.msg import Pose, Point
             
def distance_between_points(lat1, lng1, lat2, lng2):
    """
    :return: distance (meters) between two points. This is approximate as the math assumes the earth is a sphere

    https://www.movable-type.co.uk/scripts/latlong.html
    uses meters for earth radius
    """
    lat1 = math.radians(lat1)
    lng1 = math.radians(lng1)
    lat2 = math.radians(lat2)
    lng2 = math.radians(lng2)

    earth_radius = 6371000
    a = math.sin((lat2-lat1)/2)*math.sin((lat2-lat1)/2) + math.cos(lat1) * math.cos(lat2) * math.sin((lng2-lng1)/2)*math.sin((lng2-lng1)/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt((1-a)))
    distance = earth_radius * c
    return distance
def direction_between_points(lat1, lng1, lat2, lng2):
    """

    :param lat1:
    :param lng1:
    :param lat2:
    :param lng2:
    :return: bearing between two points (degrees)
    This calculates for the bearing between two points.
    """
    lat1 = math.radians(lat1)
    lng1 = math.radians(lng1)
    lat2 = math.radians(lat2)
    lng2 = math.radians(lng2)

    bearing = math.atan2((math.sin(lng2-lng1)*math.cos(lat2)),(math.cos(lat1)*math.sin(lat2)-math.sin(lat1)*math.cos(lat2)*math.cos(lng2-lng1)))
    bearing = (math.degrees(bearing)+360)%360
    return bearing
def xyz_between_points(lat1, lng1, elev1, theta1, lat2, lng2, elev2):
    """
    This is incomplete, math could be innaccurate
    """
    distance_flat = distance_between_points(lat1,lng1,lat2,lng2)
    angle = math.radians( (theta1+direction_between_points(lat1,lng1,lat2,lng2)) % 360 )
    forward = math.cos(angle)*distance_flat
    sideways = math.sin(angle)*distance_flat
    verticle = elev2-elev1
    return (forward, sideways, verticle)
def xy_between_points(lat1, lng1, theta1, lat2, lng2):
    distance_flat = distance_between_points(lat1,lng1,lat2,lng2)
    angle = math.radians( (theta1+direction_between_points(lat1,lng1,lat2,lng2)) % 360 )
    forward = math.cos(angle)*distance_flat
    sideways = math.sin(angle)*distance_flat
    return (forward, sideways)

