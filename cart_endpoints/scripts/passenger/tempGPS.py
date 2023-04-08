import rospy
from navigation_msgs.msg import LatLongPoint

class tempGPS(object):
    def __init__(self):
        rospy.init_node('tempGPS')

        gps_pose_pub = rospy.Publisher('/gps_send', LatLongPoint, queue_size=10)

        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            msg = LatLongPoint()
            msg.latitude = 10
            msg.longitude = 10
            gps_pose_pub.publish(msg)
            r.sleep()

if __name__ == "__main__":
    try:
        tempGPS()
    except rospy.ROSInterruptException:
        pass