#!/usr/bin/env python
import socket
import rospy
from sensor_msgs.msg import NavSatFix

class GPS_Parser(object):
    
    def __init__(self):
        rospy.init_node('gps_parser')
        
        self.fix_pub = rospy.Publisher('fix', NavSatFix, queue_size = 10)
        
        self.UDP_IP = "192.168.1.201"#"192.168.3.100"
        self.UDP_PORT = 8308#8308
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', self.UDP_PORT))
        #self.sock.setblocking(0)
        rospy.loginfo("Connected")
        rospy.sleep(1)
        
        while not rospy.is_shutdown():
            self.get_and_pub_packet()
        
    def get_and_pub_packet(self):
        data, addr = self.sock.recvfrom(512)
        line = data[206:278]
  
        my_fix = NavSatFix()
        my_fix.header.stamp = rospy.Time.now()

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

        my_fix.latitude = latitude
        my_fix.longitude = longitude
        my_fix.altitude = 0.0
        
        rospy.loginfo("Latitude: " + str(my_fix.latitude) + " Longitude: " + str(my_fix.longitude))

        
        
        self.fix_pub.publish(my_fix)

    def decimal_degrees(self, degs=0, mins=0, secs=0):
        return degs + (mins/60.0) + (secs/3600.0)

if __name__ == "__main__":
    GPS_Parser()

