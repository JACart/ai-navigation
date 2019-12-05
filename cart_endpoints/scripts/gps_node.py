#!/usr/bin/env python
import socket
import rospy
from sensor_msgs.msg import NavSatFix

class GPS_Parser(object):
    
    def __init__(self):
        rospy.init_node('gps_parser')
        
        self.gps_pub = rospy.Publisher('/gps_coordinates', NavSatFix, queue_size = 10)
        
        self.UDP_IP = "192.168.1.201"#"192.168.3.100"
        self.UDP_PORT = 8308#8308
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', self.UDP_PORT))
        rospy.loginfo("GPS Connected")
        r = rospy.Rate(3)
        
        while not rospy.is_shutdown():
            self.get_and_pub_packet()
            r.sleep()
        
    def get_and_pub_packet(self):
        latitude_total = 0
        longitude_total = 0
        poll_size = 10
        for x in xrange(poll_size):
            data, addr = self.sock.recvfrom(512)
            line = data[206:278]
      
            gps_coords = NavSatFix()
            gps_coords.header.stamp = rospy.Time.now()

            latitude_degrees = float(line[16:18])
            latitude_minutes = float(line[18:25])
            latitude = self.decimal_degrees(degs=latitude_degrees, mins=latitude_minutes)
            if (line[26] == 'S'):
                latitude = -1 * latitude

            longitude_degrees = float(line[28:31])
            longitude_minutes = float(line[31:38])
            longitude = self.decimal_degrees(degs=longitude_degrees, mins=longitude_minutes)
            if(line[39] == 'W'):
                longitude = -1 * longitude

            latitude_total += latitude
            longitude_total += longitude
            
            #rospy.loginfo("Latitude: " + str(gps_coords.latitude) + " Longitude: " + str(gps_coords.longitude))
            
        gps_coords.latitude = latitude_total / poll_size
        gps_coords.longitude = longitude_total / poll_size
        gps_coords.altitude = 0.0
        
        self.gps_pub.publish(gps_coords)

    def decimal_degrees(self, degs=0, mins=0, secs=0):
        return degs + (mins/60.0) + (secs/3600.0)

if __name__ == "__main__":
    GPS_Parser()

