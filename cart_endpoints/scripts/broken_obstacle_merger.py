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
        self.obstacles_in = rospy.get_param("obstacles_in", "front_camera_obstacles")
        # Name of the topic to output obstacles to
        self.obstacles_out = rospy.get_param("obstacles_out", "/test_obstacles")
        # Name of the topic to output obstacle markers to
        self.obstacle_markers_out = rospy.get_param("obstacle_markers_out", "/test_obstacle_markers")
        # Name of the coordinate frame to convert everything to
        self.target_frame = rospy.get_param("target_frame", "/base_link")
        # Size in meters of a single "tile"
        self.tile_size_m = rospy.get_param("tile_size", 0.25)
        # self.tile_size_m = rospy.get_param("tile_size", 1.0)
        # How many times per second should data be outputted from the node
        self.output_hz = rospy.get_param("output_hz", 10)
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
        rospy.loginfo("Received message from %s" % (topic));

        # Check if we need to translate it to our coordinate frame
        self.message_map[topic] = msg


    def voxelize(self):
        """
        Groups the obstacles together within their cell (voxel). Obstacles within the same 
        cell will have their locations averaged into one obstacle depending on a flag.

        @param self
        """
        local_message_map = self.message_map
        
        # Clear obstacles
        self.objects = ObstacleArray() 
        
        # vx and vy are the voxel coordinates
        # This structure keeps track of what we need to calculate the average
        # (vx, vy) -> (x_coord_sum, z_coord_sum, num_coords)
        voxel_map = {}

        for topic, msg in local_message_map.items():
            rospy.loginfo("I have %d obstacles from %s" % ( len(msg.obstacles), topic ))
            # If the message is recent enough
            # if rospy.get_rostime().to_sec() - msg.header.stamp.to_sec() < self.stale_data_sec:
                # Calculate voxel coordinates
            for obstacle in msg.obstacles:
                vx = math.floor(obstacle.pos.point.x / self.tile_size_m)
                vy = math.floor(obstacle.pos.point.y / self.tile_size_m)
                rospy.loginfo("(%f, %f, %f) obs pos (%d, %d) voxel coords" % (obstacle.pos.point.x, obstacle.pos.point.y ,obstacle.pos.point.z, vx, vy))

                # Insert the current sum into the voxel map
                key = (vx, vy)
                if self.use_center:
                    voxel_map[key] = ( vx + (self.tile_size_m / 2), vy + (self.tile_size_m / 2), 1 )
                else:
                    if not key in voxel_map:
                        voxel_map[key] = ( obstacle.pos.point.x, obstacle.pos.point.z, 1 )
                    else:
                        voxel_map[key] = ( voxel_map[key][0] + obstacle.pos.point.x, \
                                           voxel_map[key][1] + obstacle.pos.point.z, \
                                           voxel_map[key][2] + 1 )

    

        # Calculates the average of all points gathered in each voxel
        for coords, data in voxel_map.items():
            vx = coords[0]
            vy = coords[1]

            x_coord_sum = data[0]
            y_coord_sum = data[1]
            num_coords  = data[2]


            obs = Obstacle()
            obs.header.frame_id = self.target_frame
            obs.header.stamp = rospy.Time(0)
            obs.pos.point.x = x_coord_sum / num_coords
            obs.pos.point.y = y_coord_sum / num_coords
            obs.pos.point.z = 0
            obs.radius = self.tile_size_m

            rospy.loginfo("Obstacle at: (%f, %f, %f) from voxel (%d, %d)" % (obs.pos.point.x, obs.pos.point.y, obs.pos.point.z, vx, vy))

            self.objects.obstacles.append(obs)




    def local_display(self):
        """
        Publish markers for RViz to display, may appear jittery due to lack of interpolation.

        @param self
        """
        object_list = self.objects.obstacles
        for i in range(len(object_list)):
            marker = Marker()
            marker.header = Header()
            marker.header.frame_id = self.target_frame

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
