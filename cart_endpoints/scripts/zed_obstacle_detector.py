#!/usr/bin/env python
"""
Another Obstacle Detection class. Receives LaserScan data from the front facing ZED camera that has
been converted from a PointCloud. Performs the same processing as obstacle_detector.py and publishes
the obstacles to it's own topic.

In the future if more ZED cameras are added, it would be nice if each camera had a node dedicated for
its processing. This code attempted to make that easier through the use of parameters.


Note: Should be replaced with a existing in-depth object detection package.
This only detects objects within very close (3m/~9ft) vicinity of the vehicle front, 5 times a second

Zed PointCloud -> Clustering -> Circle each cluster -> Output circles
"""


import rospy
import math
import copy
import tf

# Messages
from navigation_msgs.msg import VehicleState, Obstacle, ObstacleArray
from geometry_msgs.msg import TwistStamped, Vector3, PointStamped, Point
from autoware_msgs.msg import NDTStat
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, PointCloud2

# Display purposes
from visualization_msgs.msg import Marker

class ZedObstacleDetector(object):

    def __init__(self):
        # Name of the node
        self.name = rospy.get_param("name", "front_camera_obstacle_detector")
        # Topic that laserscan data is coming from
        self.laserscan_topic_name = rospy.get_param("laserscan_in",  "/front_cam_scan")
        # Topic to output obstacles to
        self.obstacle_topic_name  = rospy.get_param("obstacles_out", "/front_cam_obstacles")
        # Topic to output obstacles markers to
        self.obstacle_markers_topic_name  = rospy.get_param("obstacle_markers_out", "/front_cam_obstacle_markers")
        # How far in front of the cart we're concerned about
        self.front_threshold = rospy.get_param("front_threshold", 8)
        # Name of the coordinate frame of the camera
        self.coordinate_frame = rospy.get_param("coordinate_frame", "/velodyne")

        # How far in front of the cart we're concerned about
        self.max_cluster_size = rospy.get_param("max_cluster_size", 70)
        self.CLOSE_BIAS = rospy.get_param("max_cluster_size", 0.5)

        rospy.init_node(self.name)

        rospy.loginfo("Node name: %s" % (self.name))
        rospy.loginfo("Laserscan Topic Name (data in):  %s" % (self.laserscan_topic_name))
        rospy.loginfo("Obstacle Topic Name  (data out): %s" % (self.obstacle_topic_name))
        rospy.loginfo("Front Threshold: %s" % (self.front_threshold))
        rospy.loginfo("Coordinate Frame: %s" % (self.coordinate_frame))

        self.t = tf.TransformListener()

        # Create global state
        self.angle_max = 0.0
        self.angle_min = 0.0
        self.angle_increment = 0.0
        
        self.processing = False
        self.curr_data = None
        self.cluster_list = []         # Raw clusters of points where obstacle may be
        self.objects = ObstacleArray() # Processed objects
        self.dist_threshold = 0.10     # Max distance between points to consider for clustering

        # TODO: Make names parameters
        self.obstacles_pub = rospy.Publisher(self.obstacle_topic_name, ObstacleArray, queue_size=10)
        self.display_pub = rospy.Publisher(self.obstacle_markers_topic_name, Marker, queue_size=10)

        self.zed_laserscan_sub = rospy.Subscriber(self.laserscan_topic_name, LaserScan, self.laserscan_callback, queue_size=1)

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # rospy.logwarn("IN WHILE LOOP")
            if self.curr_data is not None:
                # Cluster the raw LaserScan data
                self.cluster_points()

                # Put circles around the clusters of points
                self.circularize()

                self.obstacles_pub.publish(self.objects)
                rospy.loginfo("Published obstacles!")

                self.local_display(self.coordinate_frame)

                self.cluster_list = []
                self.objects = ObstacleArray()
            r.sleep()


    def laserscan_callback(self, msg):
        """
        Receives LaserScan data from prebuilt ROS node.

        @param self
        @param msg LaserScan message
        """
        rospy.loginfo("I received a LaserScan message!");
        self.curr_data = msg
        self.angle_max = msg.angle_max
        self.angle_min = msg.angle_min


    def get_point(self, angle, distance):
        """
        Converts a polar coordinate to a cartesian.
        @param self
        @param angle
        @param distance
        """
        x = math.cos(angle) * distance
        y = math.sin(angle) * distance
        return x, y


    def compare_points(self, p1, p2):
        """
        Get the Euclidean distance between two points and return true
        if the points are within "clustering distance" of each other.
        @param self
        @param p1   Point 1
        @param p2   Point 2
        """
        x1 = p1.x
        x2 = p2.x

        y1 = p1.y
        y2 = p2.y

        dist = math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )

        return dist < self.dist_threshold


    def cluster_points(self):
        """
        Clusters scans that are close enough together (within dist_threshold
        of each other).
        """
        cur_data = self.curr_data

        dists      = cur_data.ranges
        step_angle = cur_data.angle_increment
        
        # Starting angle of the LaserScan (Well at least for what we care about(only half))
        cur_angle = cur_data.angle_min #self.angle_min + ((len(arr)/2) * step_angle)
        
        cluster_list = self.cluster_list

        cur_cluster = []

        # Initialize to the beginning
        default_x, default_y = self.get_point(cur_angle, dists[0]) # Convert from "polar"
        cur_point  = Point(default_x, default_y, 0) # Unused Z coordinate
        last_point = cur_point

        for i in range(len(dists)):
            cur_angle = self.angle_min + (i * step_angle)
            # Get the X, Y coordinate from the distance and angle from front camera
            ray_dist = dists[i]

            # Only care about 8 meters ahead, limit to front 180 (TODO: Verify)
            if ray_dist <= self.front_threshold and cur_angle > -math.pi / 2 and cur_angle < math.pi / 2:
                #rospy.logwarn("We have a contender, angle: " + str(cur_angle) + "")
                curX, curY = self.get_point(cur_angle, ray_dist)

                cur_point = Point(curX, curY, 0)
                # rospy.logwarn("Current obs angle: " + str(cur_angle))
                # Cluster points that are close together or start new cluster if current cluster is getting large
                if self.compare_points(cur_point, last_point) and len(cur_cluster) < self.max_cluster_size:
                    cur_cluster.append(cur_point)
                else:
                    # Keep only significant objects
                    if len(cur_cluster) > 1:
                        cluster_list.append(cur_cluster)
                    cur_cluster = []
            
            last_point = cur_point


    def circularize(self):
        """
        Takes the cluster of points (on the same plane) and makes a sphere contain all
        of those points in the cluster for display in RViz.
        """
        self.objects = ObstacleArray()
        self.objects.header.stamp = rospy.Time.now()
        self.objects.header.frame_id = self.coordinate_frame

        for cluster in self.cluster_list:
            cur_circle = Obstacle()
            radius = 0

            first_point = cluster[0]
            last_point  = cluster[len(cluster) - 1]

            # These are local to the camera's coordinate frame
            centerX = (first_point.x + last_point.x) / 2
            centerY = (first_point.y + last_point.y) / 2

            # TODO: Make the radius of the sphere the distance from the furthest point to the center point
            radius = 0.01 * len(cluster)
            
            # Transform to world coordinate frame from camera coordinate frame
            self.t.waitForTransform("/front_cam_camera_center", self.coordinate_frame, rospy.Time(0), rospy.Duration(0.01))
            global_point = PointStamped()
            global_point.header.frame_id = self.coordinate_frame
            global_point.header.stamp = rospy.Time.now()
            global_point.point.x = centerX - self.CLOSE_BIAS
            global_point.point.y = centerY
            global_point.point.z = 0.0


            # Convert our circle from local laser coordinate space to map frame
            prepared_point = self.t.transformPoint(self.coordinate_frame, global_point)

            # Create new circle around obstacle and add to current list of obstacles
            cur_circle  = Obstacle()
            cur_circle.pos = prepared_point
            cur_circle.radius = radius
            cur_circle.followable = False
            
            self.objects.obstacles.append(cur_circle)

    #Display the obstacles on the LIDAR frame, will appear jittery in Rviz as there is no interpolation
    def local_display(self, frame):
        object_list = self.objects.obstacles
        for i in range(len(object_list)):
            marker = Marker()
            marker.header = Header()
            marker.header.frame_id = frame
            rospy.logwarn("[%s] we are in frame %s" % (self.name, frame))

            marker.ns = "Object_NS"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = 0
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.lifetime = rospy.Duration.from_sec(0.1)

            marker.pose.position.x = object_list[i].pos.point.x
            marker.pose.position.y = object_list[i].pos.point.y
            marker.pose.position.z = 0.0

            radius = object_list[i].radius
            marker.scale.x = radius
            marker.scale.y = radius
            marker.scale.z = 0.3

            self.display_pub.publish(marker)



if __name__ == "__main__":
    try:
        ZedObstacleDetector()
    except rospy.ROSInterruptException:
        pass

