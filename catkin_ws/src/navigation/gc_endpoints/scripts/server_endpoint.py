#!/usr/bin/env python 
import rospy
import requests

class server_endpoint(object):
    def __init__(self):
        rospy.init_node('server_endpoint')
        #self.cmd_p = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        #publish to waypoints topic

        getGoal()

        rospy.spin()

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
