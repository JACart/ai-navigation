#!/usr/bin/env python 
import rospy
import requests
from navigation_msgs.msg import LatLongPoint, WaypointsArray
from sensor_msgs.msg import NavSatFix
import datetime
"""
This node is used to interface with the server to grab intermediary waypoints and other relevant information.
This node should also post to the server with information that goes to the frontend.
"""

server_ip = "134.126.153.21"
server_port = "5000"


class server_endpoint(object):

    def __init__(self):
        rospy.init_node('server_endpoint')
        
        #publish to waypoints topic
        self.waypoint_pub = rospy.Publisher('/waypoints', WaypointsArray, queue_size = 10, latch=True)
        
        
        
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
        
        #subscribe to various sensor topics (in order to post that data back to the server for frontend)
        """
        subscriber for velocity
        subscriber for camera
        subscriber for lightware
        subscriber for velodyne
        subscriber for rplidar
        """
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
            have_goal = self.clear_goals()
            r.sleep()

    def get_goals(self):
        url = 'http://'+server_ip+':'+server_port+'/goals'
        r = requests.get(url)

        if not r.ok:
            print "Error connecting to API"
            rospy.logerr("Server_endpoint: Error connecting to API")
            return r.ok

        self.goals = r.json()
        return self.goals['Goals'] != []

    def parse_goals(self):
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

    def clear_goals(self):
        url = 'http://'+server_ip+':'+server_port+'/clear'
        r = requests.get(url)
        return not r.ok

    def send_status(self):
        """
        Sends POST request to /cardata on server. This will be called by each callback whenever new data is received on that callback
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
        
        cardata["timestamp"] = str(datetime.datetime.now())
        
        payload = {}
        payload["newData"] = cardata

        r = requests.post(url, json= payload, headers={ 'Content-Type': 'application/json',})
        if not r.ok:
            print "Error making post request: "+str(r.status_code)
            
    def gps_callback(self, gps_data):
        """
        Sets lat and long using data from GPS
        
        gps_data is a NavSatFix msg
        """
        lat = gps_data.latitude
        lon = gps_data.longitude
        if (not self.lat == lat) or (not self.lon == lon):
            print "New GPS Found, data being posted to server"
            self.lat = lat
            self.lon = lon
            self.send_status()
            
if __name__ == "__main__":   
    try:
        server = server_endpoint()
    except rospy.ROSInterruptException:
	pass