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

        self.get_locations()
        #self.send_status()
        rospy.spin()

    def get_locations(self):
        """
        The waypoints in the actual server are not set up yet. I used this as test code for publishing to the waypoints topic.
        This code will need to be updated once the server is.
        """
        url = 'http://'+server_ip+':'+server_port+'/locations'
        print url
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
        waypoints.waypoints = w_list
        self.waypoint_pub.publish(waypoints)
        print "published"

    def get_waypoints(self):
        pass
    def send_status(self):
        """
        Attempted to try a post but posts were not allowed on the server. Same applied for put.
        """
        url = 'http://'+server_ip+':'+server_port+'/cardata'
        r = requests.get(url)
        print "Get request: "
        print r.json()
        #All of the payload values need to be set based on subscribers to topics. None of this should be hardcoded in the end
        
        payload = {}
        payload['Cardata'] = []
        cardata = {}
        cardata['battery'] = 37.3
        cardata['camera'] = "BAD"
        cardata['gps'] = "3.776,-70.2212"
        cardata['lightware'] = "BAD"
        cardata['rplidar'] = "GOOD"
        cardata['velocity'] = 3.6
        cardata['velodyne'] = "GOOD"
        payload['Cardata'].append(cardata)
        print "Payload: "
        print unicode(payload)
        print "Post request: "
        r = requests.post(url, data = unicode(payload))
        print r.json()
        
def getGoal():
    r = requests.get('https://practice-jad006.c9users.io/quotations')

    print r.status_code
    print r.ok

    print r.headers['content-type']

    #print r.text

    #print r.json()


    return r.ok



	
if __name__ == "__main__":
    try:
	server_endpoint()
    except rospy.ROSInterruptException:
	pass
