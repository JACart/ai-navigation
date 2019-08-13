#!/usr/bin/env python

import datetime
import requests
import rospy
from navigation_msgs.msg import WaypointsArray
from sensor_msgs.msg import NavSatFix
"""
This node is used to interface with the server to grab intermediary waypoints and
other relevant information.
This node should also post to the server with information that goes to the frontend.
"""

SERVER_IP = "134.126.153.21"
SERVER_PORT = "5000"


class ServerEndpoint(object):

    def __init__(self):
        rospy.init_node('server_endpoint')

        #publishers
        self.waypoint_pub = rospy.Publisher('/waypoints', WaypointsArray,
                                            queue_size=10, latch=True)

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
        self.waypoint_list = None

        ''' SEND STATUS IN THE CALLBACK OF EACH UPDATE! '''
        self.goals = None
        have_goal = False

        '''subscribe to various sensor topics (in order to post that data back to the server
        for frontend)'''

        self.gps_sub = rospy.Subscriber('/fix', NavSatFix, self.gps_callback)

        rospy.sleep(1)
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            #loop until we get a goal from the api
            while not have_goal:
                have_goal = self.get_goals()
                r.sleep()


            print "Goals recieved"
            self.parse_goals()
            have_goal = clear_goals()
            r.sleep()

    def get_goals(self):
        url = 'http://'+SERVER_IP+':'+SERVER_PORT+'/goals'
        r = requests.get(url)

        if not r.ok:
            print "Error connecting to API"
            rospy.logerr("Server_endpoint: Error connecting to API")
            return r.ok

        self.goals = r.json()
        return self.goals['Goals'] != []

    def parse_goals(self):
        """
        For every goal create a waypoint and add to a list
        """
        waypoints = WaypointsArray()
        w_list = []
        for location in self.goals['Goals']:
            current = NavSatFix()
            current.latitude = float(location['lat'])
            current.longitude = float(location['long'])
            current.altitude = float(location['elev'])
            w_list.append(current)

        waypoints.waypoints = w_list
        self.waypoint_pub.publish(waypoints)

    def send_status(self):
        """
        Sends POST request to /cardata on server. This will be called by each callback
        whenever new data is received on that callback
        """
        url = 'http://'+SERVER_IP+':'+SERVER_PORT+'/cardata'
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
        cardata["timestamp"] = str(datetime.datetime.now())

        payload = {}
        payload["newData"] = cardata

        r = requests.post(url, json=payload, headers={'Content-Type': 'application/json',})
        if not r.ok:
            print "Error making post request: "+str(r.status_code)

    def gps_callback(self, gps_data):
        """
        Sets lat and long using data from GPS
        gps_data is a NavSatFix msg
        """
        lat = gps_data.latitude
        lon = gps_data.longitude
        if self.lat != lat or self.lon != lon:
            print "New GPS Found, data being posted to server"
            self.lat = lat
            self.lon = lon
            self.send_status()
def clear_goals():
    url = 'http://'+SERVER_IP+':'+SERVER_PORT+'/clear'
    r = requests.get(url)
    return not r.ok

if __name__ == "__main__":
    try:
        ServerEndpoint()
    except rospy.ROSInterruptException:
        pass
