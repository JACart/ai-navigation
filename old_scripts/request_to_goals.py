#!/usr/bin/env python
"""
This code takes a front end request message, and contacts googlemaps api to recieve a polyline of a valid path. 
This will likely be unused in the final design.
"""

import sys
import rospy

import requests
import csv
import json
from googlemaps import client as client
import math
import polyline

from geometry_msgs.msg import Pose, Point
from navigation_msgs.msg import fe_request, lat_long_point, list_of_goals

google_key = ""
google_elevation_key = ""

class GoalNode(object):
    
    def __init__(self):
        """ Set up the node. """
        
        rospy.init_node('goal_node')
        
        self.goal_subscriber = rospy.Subscriber('/fe_request', fe_request, self.goal_callback, queue_size = 10)
        self.goal_publisher = rospy.Publisher('/list_of_goals', list_of_goals, queue_size = 10)
        
        rospy.spin()

    def goal_callback(self, request):
        print "Starting api test"
        starting_location = request.starting_location
        goal_location = request.final_location
        data = self.get_google_locations(starting_location, goal_location)
        print "Google polyline recieved"
        print str(data)
        print "Getting elevation data"
        data_elev = self.get_elevations(data)
        print "elevation data received"
        print "Formulating ros message"
        output = list_of_goals()
        for point in data_elev:
            new_goal = lat_long_point()
            new_goal.latitude = point[0]
            new_goal.longitude = point[1]
            new_goal.elevation = point[2]
            output.goals.append(new_goal)
        
        self.goal_publisher.publish(output)
        
    def get_google_locations(self, start_string, end_string, path=False):
        """
        :param start_string: starting location
        :param end_string: destination location
        :param path: inid
        :return:
        """
        #set whether or not to use a pedestrian path
        mode = "driving"
        if path:
            mode = "walking"
    
        use_client = client.Client(key=google_key)
    
        #Example responses to test code with (to limit api call count)
        #directions = dict([{u'overview_polyline': {u'points': u'cgriFt|k`NFF?E@Md@aALi@Do@IkBGa@}@sAQk@E]?g@B[Lk@LWNQTO`@K|AGb@?lAJjDZXFVoBBmAE{@WoAsAqFWo@Im@Cg@Ba@Fa@r@{BnAeEFy@CsA[cBAq@Hq@Tw@'}, u'warnings': [], u'bounds': {u'northeast': {u'lat': 38.4373241, u'lng': -78.86271409999999}, u'southwest': {u'lat': 38.4346274, u'lng': -78.8732684}}, u'waypoint_order': [], u'summary': u'Bluestone Dr and Carrier Dr', u'copyrights': u'Map data \xa92018 Google', u'legs': [{u'distance': {u'text': u'0.8 mi', u'value': 1273}, u'traffic_speed_entry': [], u'end_address': u'701 Carrier Dr, Harrisonburg, VA 22807, USA', u'via_waypoint': [], u'start_address': u'Wilson Hall, 951 Madison Dr, Harrisonburg, VA 22801, USA', u'start_location': {u'lat': 38.4371436, u'lng': -78.87323119999999}, u'steps': [{u'html_instructions': u'Head <b>southwest</b> on <b>Madison Dr</b> toward <b>Bluestone Dr</b>', u'distance': {u'text': u'20 ft', u'value': 6}, u'travel_mode': u'DRIVING', u'start_location': {u'lat': 38.4371436, u'lng': -78.87323119999999}, u'polyline': {u'points': u'cgriFt|k`NFF'}, u'duration': {u'text': u'1 min', u'value': 1}, u'end_location': {u'lat': 38.4371032, u'lng': -78.8732684}}, {u'html_instructions': u'Turn <b>left</b> onto <b>Bluestone Dr</b><div style="font-size:0.9em">Closed Mon\u2013Fri 7:00 AM \u2013 7:00 PM</div>', u'distance': {u'text': u'0.4 mi', u'value': 571}, u'travel_mode': u'DRIVING', u'maneuver': u'turn-left', u'start_location': {u'lat': 38.4371032, u'lng': -78.8732684}, u'polyline': {u'points': u'{friF||k`N?E?E@GBEP]HODMBIDQBM@I@C@K?U@KA]C]Ec@?KAECEAIACCECCQWOQAAGKGMCEGSCGAIAOAA?EAE?C?IAI@O?E?C@G@I@I@IBKDK@C@EBEDG@CDE@ADEDEDCFE@?DCDABAPCH?DAJA`ACT?L?F?dAJnCTZDH@ND'}, u'duration': {u'text': u'2 mins', u'value': 147}, u'end_location': {u'lat': 38.4347736, u'lng': -78.8701761}}, {u'html_instructions': u'Turn <b>left</b> onto <b>Carrier Dr</b><div style="font-size:0.9em">Destination will be on the right</div>', u'distance': {u'text': u'0.4 mi', u'value': 696}, u'travel_mode': u'DRIVING', u'maneuver': u'turn-left', u'start_location': {u'lat': 38.4347736, u'lng': -78.8701761}, u'polyline': {u'points': u'ixqiFrik`NBWJo@D[@K@M@[?Y?I?IA[AKAIAKAEG[Ka@EUc@eBUeAGWCGCGACACGOEGEMCIAICIAOAIAI?EAC?S?C?E@I@M?E@IBI@GBI^iA@CLc@FS@C|@{CFQ@M@I@O@Q?Q?SAMA_@Os@COCQCM?MAI?K?M@K@O@KBIBMBIBKHS'}, u'duration': {u'text': u'2 mins', u'value': 119}, u'end_location': {u'lat': 38.4346365, u'lng': -78.86271409999999}}], u'duration': {u'text': u'4 mins', u'value': 267}, u'end_location': {u'lat': 38.4346365, u'lng': -78.86271409999999}}]}])
        #overview_points = "cgriFt|k`NFF?E@Md@aALi@Do@IkBGa@}@sAQk@E]?g@B[Lk@LWNQTO`@K|AGb@?lAJjDZXFVoBBmAE{@WoAsAqFWo@Im@Cg@Ba@Fa@r@{BnAeEFy@CsA[cBAq@Hq@Tw@"
    
        #Real lines to get polyline from googles servers
        directions = client.directions(use_client, start_string, end_string, transit_mode=mode)
        overview_points = directions[0]["overview_polyline"]["points"]
    
        points = polyline.decode(overview_points) #uses polyline package to decode.
        return points
    def get_elevations(self, points):
        """
        :param points: list of points (lat/long tuple)
        :return: list of lat/long/elevation tuples
        """
        use_client = client.Client(key=google_elevation_key)
        elevations = client.elevation(use_client, points)
        new_list = []
        for elevation in elevations:
            new_list.append((elevation["location"]["lat"], elevation["location"]["lng"], elevation["elevation"]))
    
        return new_list
        
if __name__ == "__main__":
    try:
        GoalNode()
    except rospy.ROSInterruptException:
        pass

