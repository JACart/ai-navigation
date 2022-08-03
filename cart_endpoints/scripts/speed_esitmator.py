





class SpeedEstimator(object):
    
    def __init__(self):

        self.last_msg = None


    def receiveGPS(self, msg):

        if self.last_msg == None:
            self.last_msg = msg
            return

        # Check times msg.header.stamp.to_sec()

        rospy.loginfo(msg.latitude)
        rospy.loginfo(msg.longitude)
        rospy.loginfo(msg.elevation)
        


if __name__ == "__main__":
    try:
        SpeedEstimator()
    except rospy.ROSInterruptException:
        pass
