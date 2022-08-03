#!/usr/bin/env python
'''
Obstacle Merger class. Takes in many lists of obstacles and combines them to remove duplicates of similar
obstacles for detectors with overlapping fields of view (i.e. LiDAR and ZED see the same person).

The best way to picture this calculation in your head is to imagine the obstacles over a tile floor.
All the readings that are on the same tile get collapsed into the same obstacle located at the center
of the tile (TODO: Consider averaging them together). This also implies that this node will only merge
obstacles on the world's XZ plane since we do not care about the height.

Obstacle coordinates will all be translated to the same coordinate frame (default := base_link)
before they are collapsed.

TODO: Do some ascii art describing the process

+--------------------------------------------------+
|                                                  |
|                                                  |
|                                                  |
|                                                  |
|                                                  |
|                                                  |
+--------------------------------------------------+


'''


import rospy
import numpy as np
import math
import tf

# Messages
from navigation_msgs.msg import VehicleState, Obstacle, ObstacleArray
from geometry_msgs.msg import TwistStamped, Vector3, PointStamped
from std_msgs.msg import Header

# Display
from visualization_msgs.msg import Marker



class ObstacleMerger(object):
    
    def __init__(self):
        
        # ----- Constants -----
        self.NS_TO_SEC = 1 / (10 ** 9)

        # ----- Parameters -----
        # Name of the node
        self.name = rospy.get_param("name", "obstacle_merger")
        # Space delimited list of topics that publish ObstacleArray's for input
        # self.obstacles_in = rospy.get_param("obstacles_in", "lidar_obstacles front_cam_obstacles front_cam_obj_obstacles /ransac_obtacles")
        self.obstacles_in = rospy.get_param("obstacles_in", "lidar_obstacles front_cam_obstacles front_cam_obj_obstacles ransac_obstacles")
        # Name of the topic to output obstacles to
        self.obstacles_out = rospy.get_param("obstacles_out", "/obstacles")
        # Name of the topic to output obstacle markers to
        self.obstacle_markers_out = rospy.get_param("obstacle_markers_out", "/obstacle_markers")
        # Name of the coordinate frame to convert everything to
        self.target_frame = rospy.get_param("target_frame", "/velodyne")
        # Size in meters of a single "tile"
        self.tile_size_m = rospy.get_param("tile_size", 0.25)
        # self.tile_size_m = rospy.get_param("tile_size", 1.0)
        # How many times per second should data be outputted from the node
        self.output_hz = rospy.get_param("output_hz", 20)
        # How long should we keep considering old data in seconds
        self.stale_data_sec = rospy.get_param("stale_data_sec", 1.0)        
        # Whether or not you should use the center of the tile or average the points together
        self.use_center = rospy.get_param("use_center", True)        

        if self.tile_size_m == 0:
            rospy.logfatal("Tile size cannot be 0!")

        rospy.init_node(self.name)

        rospy.loginfo("obstacles_in: '%s'" % (self.obstacles_in))

        # ----- Node State -----
        # Maps the topic name to the latest message received from that topic
        self.message_map = {}
        # Maps the topic name to the subscriber object
        self.subscribers = {}
        # Array of objects we want to publish
        self.objects = ObstacleArray() 
        # Publishers for output topics
        self.obstacles_pub = rospy.Publisher(self.obstacles_out, ObstacleArray, queue_size=10)
        self.display_pub = rospy.Publisher(self.obstacle_markers_out, Marker, queue_size=10)
        # Allows you to transform points to different coordinate frames
        self.t = tf.TransformListener()


        # Check if we have any input topics
        if not self.obstacles_in:
            rospy.logerr("Not subscribing to any topics!")
        else:
            # Subscribe to all of the input messages
            for topic in self.obstacles_in.split(" "):
                rospy.loginfo("Subscribing to %s" % (topic))
                self.subscribers[topic] = rospy.Subscriber(topic, ObstacleArray, \
                                              callback=self.receiveObstacleArray, \
                                              callback_args=topic, \
                                              queue_size=10)


        # Repeatedly merge obstacles and publish results until shutdown
        r = rospy.Rate(self.output_hz)
        while not rospy.is_shutdown():
            self.voxelize()
            self.obstacles_pub.publish(self.objects)
            self.local_display()
            r.sleep()


    def receiveObstacleArray(self, msg, topic):
        """
        Receives ObstacleArray data from a topic we've subscribed to.

        Converts the message to our desired coordinate frame.

        @param self
        @param msg   ObstacleArray message
        @param topic Topic that the message came from
        """
        #rospy.loginfo("Received message from %s" % (topic));

        if not msg.header.frame_id:
            rospy.logwarn("[%s] frame_id of message from %s is: '%s'" % (self.name, topic, msg.header.frame_id))

        # Check if we need to translate it to our coordinate frame
        # if msg.header.frame_id and msg.header.frame_id != self.target_frame:
        #     tx = 0.0
        #     ty = 0.0
        #     tz = 0.0
        #     try:
        #         (trans,rot) = self.t.lookupTransform(self.target_frame, msg.header.frame_id, rospy.Time(0))
        #         tx = trans[0]
        #         ty = trans[1]
        #         tz = trans[2]
        #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #         rospy.logerr("[%s] Failed to translate obstacle from %s to %s!" % \
        #             (self.name, msg.header.frame_id, self.target_frame))

        #     rospy.logwarn("[%s] Performed translation of (%f, %f, %f)!" % (self.name, tx, ty, tz))
        #     for obstacle in msg.obstacles:
        #         obstacle.pos.point.x += tx
        #         obstacle.pos.point.y += ty
        #         obstacle.pos.point.z += tz
        #         obstacle.header.frame_id = self.target_frame
            
        #     msg.header.frame_id = self.target_frame

                
        # If the message isn't stamped, stamp it ourselves
        if msg.header.stamp == rospy.Time(0):
            msg.header.stamp = rospy.Time.now()
            rospy.logwarn("[%s] Time was not set in message from '%s'!" % (self.name, topic))

        self.message_map[topic] = msg


    def voxelize(self):
        """
        TODO: Groups the obstacles together within their cell (voxel). Obstacles within the same 
        cell will have their locations averaged into one obstacle depending on a flag.

        Just concatenates all of the obstacles together.

        @param self
        """
        local_message_map = self.message_map
        
        # Clear obstacles
        self.objects = ObstacleArray() 

        for topic, msg in local_message_map.items():
            if rospy.Time.now().to_sec() - msg.header.stamp.to_sec() < self.stale_data_sec:
                for obs in msg.obstacles:
                    self.objects.obstacles.append(obs)
            else:
                rospy.logwarn("[%s] Stale data in message map for '%s'! Did you stop publishing?" % (self.name, topic))



    def local_display(self):
        """
        Publish markers for RViz to display, may appear jittery due to lack of interpolation.

        @param self
        """
        object_list = self.objects.obstacles
        for i in range(len(object_list)):
            marker = Marker()
            marker.header = Header()
            # rospy.logwarn("[%s] I have an obstacle in frame '%s'!" % (self.name, object_list[i].header.frame_id))
            marker.header.frame_id = object_list[i].header.frame_id

            marker.ns = "Object_NS"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = 0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration.from_sec(0.1)

            marker.pose.position.x = object_list[i].pos.point.x
            marker.pose.position.y = object_list[i].pos.point.y
            marker.pose.position.z = 0.0

            radius = object_list[i].radius
            marker.scale.x = radius
            marker.scale.y = radius
            marker.scale.z = 0.1

            self.display_pub.publish(marker)






if __name__ == "__main__":
    try:
        ObstacleMerger()
    except rospy.ROSInterruptException:
        pass
