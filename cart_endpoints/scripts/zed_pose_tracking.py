"""
This ROS node checks to see if any people in the cart are inside of some bounding box.
I am not sure what the problem is but with the current ZED situation, but when we actually use the passenger camera we completly drop all the information and will sometimes lose the
camera data.

@author Jacob McClaskey
@version 4/20/2022
"""

import rospy

# Messages
from navigation_msgs.msg import Obstacle, ObstacleArray
from std_msgs.msg import Header
from zed_interfaces.msg import ObjectsStamped

# Display purposes
from visualization_msgs.msg import Marker

class Zed_Pose_Tracking(object):
    def __init__(self):
        
        # ----- Parameters -----
        # Name of the node
        self.name = rospy.get_param("name", "passenger_cam_passangers_to_check")
        # Name of the topic that subscribes to the ZED objects
        # self.objects_in = rospy.get_param("objects_in", "/passenger_cam/passenger/obj_det/objects")
        self.objects_in = rospy.get_param("objects_in", "/front_cam/front/obj_det/objects")
        # Name of the topic that publishes the obstacles
        self.obstacle_markers_out = rospy.get_param("passenger_markers_out", "/passenger_obj_location")
        # self.stop_pub = rospy.Publisher('/stop', Stop, queue_size=10)

        rospy.init_node(self.name)

        # Assume everything is ok
        self.passenger_safe = True
        self.passenger_list = None
        self.time_seen = None
        self.frame = None

        self.vehicle_width = rospy.get_param('vehicle_width', 1.1938)
        self.vehicle_length = rospy.get_param('vehicle_length', 2.4003)
        self.wheel_base = rospy.get_param('wheel_base', 2.4003)
        self.front_axle_track = rospy.get_param('front_axle_track', .9017)
        self.rear_axle_track = rospy.get_param('rear_axle_track', .9652)
        self.tire_width = rospy.get_param('tire_width', .2159)
        
        
        # Vehicle Corners (Wheel Positions)
        self.front_right_corner = [0, 0]
        self.front_left_corner = [0, 0]
        self.rear_left_corner = [0, 0]
        self.rear_right_corner = [0, 0]
        self.front_axle_center = [0, 0]
        self.obtain_corners()



        # ----- Node State -----
        self.object_sub = rospy.Subscriber(self.objects_in, ObjectsStamped, callback=self.receiveObjects, queue_size=10)
        self.display_pub  = rospy.Publisher(self.obstacle_markers_out, Marker, queue_size=10)

        # only wanna check positions 5 times a second due to obj_dect not great
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.passenger_list is not None:
                self.pose_track()
            r.sleep()


    def pose_track(self):
        cur_pass = self.passanger_list
        for passenger in cur_pass:
            #      1 ------- 2
            #     /.        /|
            #    0 ------- 3 |
            #    | .       | |
            #    | 5.......| 6
            #    |.        |/
            #    4 ------- 7
            # index for box check
            box = passenger.bounding_box_3d
            rospy.logwarn("Checking passenger location")
            # rospy.logwarn("Passenger bounding box 0: %s, 3: %s", box.corners[0].kp, box.corners[3].kp)
            # rospy.logwarn("cart left corner: %s cart right corner: %s", self.front_left_corner, self.front_right_corner)
            if box.corners[0].kp[1] < self.front_left_corner[1]:
                rospy.logwarn("Passenger bounding box outside the left corner cart demensions")
                # rospy.logwarn("cart left corner: %s cart right corner: %s", self.front_left_corner, self.front_right_corner)
            if box.corners[3].kp[1] > self.front_right_corner[1]: # check this is wrong 
                rospy.logwarn("Passenger bounding box outside the right corner cart demensions")


    def receiveObjects(self, msg):
        """
        Receives an ObjectsStamped message and saves them to be passanger_list.
        Might want to turn all this information into a msg but not worrying right now

        @param self 
        @param msg  ObjectsStamped message
        """
        self.frame = msg.header.frame_id
        self.time = msg.header.stamp
        self.passanger_list = msg.objects
        self.pose_track()
        # self.local_display("/passenger_cam_left_camera_frame", self.passanger_list)
        self.local_display("/front_cam_left_camera_frame", self.passanger_list)
        

    # Display the passenger as a sphere
    def local_display(self, frame, passanger_list):
        for i in range(len(passanger_list)):
            marker = Marker()
            marker.header = Header()
            marker.header.frame_id = frame

            marker.ns = "Object_NS"
            marker.id = passanger_list[i].label_id
            marker.type = Marker.SPHERE
            marker.action = 0
            marker.color.r = 0.0
            marker.color.g = 0.8
            marker.color.b = 0.9
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration.from_sec(0.5)

            marker.pose.position.x = passanger_list[i].position[0]
            marker.pose.position.y = passanger_list[i].position[1]
            marker.pose.position.z = 0.0
            # marker.pose.position.x = self.front_left_corner[0]
            # marker.pose.position.y = self.front_left_corner[1]
            # marker.pose.position.z = 0.0

            radius = max(passanger_list[i].position[0], passanger_list[i].position[1])
            marker.scale.x = radius/2
            marker.scale.y = radius/2
            marker.scale.z = radius/2

            self.display_pub.publish(marker)


    def obtain_corners(self):
        """ This could be simplified to simply obtaining the position using the tf library
        Point of experimenting in the next couple days, not sure which is more flexible yet.
        """

        self.rear_left_corner = [-(self.wheel_base/2), ((self.front_axle_track/2) + (self.tire_width/2))]
        self.rear_right_corner = [-(self.wheel_base/2), -((self.front_axle_track/2) + (self.tire_width/2))]
        self.front_left_corner = [(self.wheel_base/2), -((self.rear_axle_track/2) + (self.tire_width/2) + 0.3)]
        self.front_right_corner= [(self.wheel_base/2), ((self.rear_axle_track/2) + (self.tire_width/2) - 0.3)]
        self.front_axle_center = [(self.wheel_base/2), 0]


if __name__ == "__main__":
    try:
        Zed_Pose_Tracking()
    except rospy.ROSInterruptException:
        pass
