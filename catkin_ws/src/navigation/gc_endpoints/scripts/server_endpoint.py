#!/usr/bin/env python 
import rospy
import requests
from navigation_msgs.msg import LatLongPoint, WaypointsArray
from sensor_msgs.msg import NavSatFix
"""
This node is used to interface with the server to grab intermediary waypoints and other relevant information.
This node should also post to the server with information that goes to the frontend.
"""

server_ip = "134.126.153.21"
server_port = "5000"


class server_endpoint(object):

    def __init__(self):
        rospy.init_node('server_endpoint')
        #self.cmd_p = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        
        #publish to waypoints topic
        self.waypoint_pub = rospy.Publisher('/waypoints', WaypointsArray, queue_size = 10, latch=True)
        #subscribe to various sensor topics (in order to post that data back to the server for frontend)
        """
        subscriber for gps data
        subscriber for velocity
        subscriber for camera
        subscriber for lightware
        subscriber for velodyne
        subscriber for rplidar
        """
        #variables to be set by callbacks
        self.battery = 0.0
        self.camera = "No Data Yet"
        self.elevation = 0.0
        self.lat = 0.0
        self.lon = 0.0
        self.lightware = "No Data Yet"
        self.rplidar = "No Data Yet"
        self.velocity = 0.0
        self.velodyne = "No Data Yet"

        rospy.sleep(1)
        r = rospy.Rate(2)
        self.waypoint_list = None
        while not rospy.is_shutdown():
            self.get_waypoints()
            self.send_status()
            r.sleep()
        rospy.spin()

    def get_waypoints(self):
        """
        The waypoints in the actual server are not set up yet. I used this as test code for publishing to the waypoints topic.
        This code will need to be updated once the server is.
        """
        url = 'http://'+server_ip+':'+server_port+'/locations'
        r = requests.get(url)
        locations = r.json()
        waypoints = WaypointsArray()
        w_list = []
        for location in locations['Locations']:
            current = NavSatFix()
            current.latitude = float(location['lat'])
            current.longitude = float(location['long'])
            current.altitude = float(0)
            w_list.append(current)
        #if len(waypoints.waypoints) == 0:
        #    return
        
        if not str(w_list) == str(self.waypoint_list):
            self.waypoint_list = w_list
            waypoints.waypoints = w_list
            self.waypoint_pub.publish(waypoints)
    def send_status(self):
        """
        Attempted to try a post but posts were not allowed on the server. Same applied for put.
        """
        url = 'http://'+server_ip+':'+server_port+'/cardata'
        r = requests.get(url)

        cardata = {}
        cardata["battery"] = self.battery
        cardata["camera"] = self.camera
        cardata["elevation"] = self.elevation
        cardata["lat"] = self.lat
        cardata["lon"] = self.lon
        cardata["lightware"] = self.lightware
        cardata["rplidar"] = self.rplidar
        cardata["velocity"] = self.velocity
        cardata["velodyne"] = self.velodyne
        
        payload = {}
        payload["newData"] = cardata
        
        #json_headers =   
        r = requests.post(url, json= payload, headers={ 'Content-Type': 'application/json',})
        if not r.ok:
            print "Error making post request: "+str(r.status_code)
	
if __name__ == "__main__":
    try:
	server_endpoint()
    except rospy.ROSInterruptException:
	pass
