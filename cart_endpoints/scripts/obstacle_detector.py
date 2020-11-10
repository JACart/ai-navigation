#!/usr/bin/env python
'''
Obstacle Detection class. Takes raw laserscan data from 2D RPLIDAR on the front of the cart.
Processes the data and determines where obstacles may be. The obstacles are then passed to a local planner node
that determines what to do with those obstacles if action must be taken(avoid, stop, etc).

Note: Should be replaced with a existing in-depth object detection package.
This only detects objects within very close (3m/~9ft) vicinity of the vehicle front, 5 times a second

LiDAR Data -> Clustering -> Circle each cluster -> Output circles
'''
import rospy
import math
import copy
import tf

#Messages
from navigation_msgs.msg import EmergencyStop, VehicleState, Obstacle, ObstacleArray
from geometry_msgs.msg import TwistStamped, Vector3, PointStamped
from autoware_msgs.msg import NDTStat
from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, PointCloud2

#Display purposes
from visualization_msgs.msg import Marker

class Point(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def getX(self):
        return self.x

    def getY(self):
        return self.y

class ObstacleDetector(object):

    def __init__(self):
        rospy.init_node('obstacle_detector')

        self.t = tf.TransformListener()

        #Constants
        self.angle_max = 3.14159274101
        self.angle_min = 0.0
        self.angle_increment = 0.0

        #Most recent laser scan data from 2d LiDAR
        self.curr_lidar_data = None

        self.processing = False

        #Raw clusters of points where obstacle may be
        self.cluster_list = []

        #Processed objects
        self.objects = ObstacleArray()

        # Max distance between points to consider for clustering
        self.dist_threshold = 0.10
        
        self.emergency_stop_pub = rospy.Publisher('/emergency_stop', EmergencyStop, queue_size=10)
        self.obstacles_pub = rospy.Publisher('/obstacles', ObstacleArray, queue_size=10)
        self.display_pub = rospy.Publisher('/obstacle_display', Marker, queue_size=10)
        
        self.rplidar_sub = rospy.Subscriber('/scan_rplidar', LaserScan, self.lidar_callback, queue_size=1)
        self.velodyne_laserscan_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback, queue_size=1)
        # /scan is the topic the pointcloud_to_laserscan node publishes to

        #self.vehicle_speed_sub = rospy.Subscriber('/estimate_twist', TwistStamped, self.speed_check)
        
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.curr_lidar_data is not None:
                #Cluster the raw LaserScan data
                self.cluster_points()

                #Put circles around the clusters of points
                self.circularize()

                self.obstacles_pub.publish(self.objects)

                self.local_display("/base_link")

                self.cluster_list = []
                self.objects = ObstacleArray()
            r.sleep()

    #Update current LiDAR data
    def lidar_callback(self, msg):
        self.curr_lidar_data = msg
        self.angle_max = msg.angle_max
        self.angle_min = msg.angle_min

    # Basically polar to cartesian
    def get_point(self, angle, distance):
        #angle = math.degrees(angle)
        #rospy.loginfo("Angle: " + str(angle))

        y = math.sin(angle) * distance
        x = math.cos(angle) * distance

        #rospy.loginfo("x in feet: " + str(x*3.281))
        #rospy.loginfo("y in feet: " + str(y*3.281))

        return x, y

    #Just distance formula and return if the points are within "clustering distance" of each other
    def compare_points(self, p1, p2):
        x1 = p1.getX()
        x2 = p2.getX()

        y1 = p1.getY()
        y2 = p2.getY()

        dist = math.sqrt( (x2-x1)**2 + (y2-y1)**2 )

        return dist < self.dist_threshold
    
    def cluster_points(self):
    #LidarScan message we will process
        cur_data = self.curr_lidar_data

        #Get a handle on the distances output by LiDAR
        arr = cur_data.ranges

        #Angle increase between each emasurement
        step_angle = cur_data.angle_increment
        
        #Starting angle of the LiDAR (Well at least for what we care about(only half))
        cur_angle = cur_data.angle_min #self.angle_min + ((len(arr)/2) * step_angle)
        
        cluster_list = self.cluster_list

        cur_cluster = []

        # Initialize to the beginning
        
        default_x, default_y = self.get_point(cur_angle, arr[0])
        cur_point = Point(default_x, default_y) 
        last_point = cur_point

        for i in range(len(arr)):
            cur_angle = self.angle_min + (i * step_angle)
            # Get the X, Y coordinate from the distance and angle form LiDAR
            ray_dist = arr[i]

            # Only care about 3 meters ahead, limit to front 180
            if ray_dist <= 8 and ( (cur_angle < (-math.pi/2)) or (cur_angle > (math.pi/2)) ):
                #rospy.loginfo("We have a contender, angle: " + str(cur_angle) + "")
                curX, curY = self.get_point(cur_angle, ray_dist)

                cur_point = Point(curX, curY)

                #Cluster points that are close together, also start new cluster if current cluster is getting large
                if self.compare_points(cur_point, last_point) and len(cur_cluster) < 70:
                    cur_cluster.append(cur_point)
                else:
                    #Keep only significant objects
                    if len(cur_cluster) > 1:
                        cluster_list.append(cur_cluster)
                    cur_cluster = []
            
            last_point = cur_point

    #Put circles around our clusters(really segments)
    def circularize(self):
        self.objects = ObstacleArray()

        for cluster in self.cluster_list:
            cur_circle = Obstacle()
            radius = 0

            first_point = cluster[0]
            last_point = cluster[len(cluster) - 1]

            #Local to the base_laser_link
            centerX = (first_point.getX() + last_point.getX()) / 2
            centerY = (first_point.getY() + last_point.getY()) / 2

            #avg point spacing is maybe ~3 inches? I will change this later to a more formal method
            radius = 0.01 * len(cluster)
            
            #Transform to another frame, in our case map
            self.t.waitForTransform("/base_link", "/base_laser_link", rospy.Time(0), rospy.Duration(0.01))
            global_point = PointStamped()
            global_point.header.frame_id = "base_laser_link"
            global_point.header.stamp = rospy.Time(0)
            global_point.point.x = centerX
            global_point.point.y = centerY
            global_point.point.z = 0.0


            #Convert our circle from local laser coordinate space to map frame
            prepared_point = self.t.transformPoint("base_link", global_point)

            #Create new circle around obstacle and add to current list of obstacles
            cur_circle  = Obstacle()
            cur_circle.pos = prepared_point
            cur_circle.radius = radius
            
            self.objects.obstacles.append(cur_circle)

    #Display the obstacles on the LIDAR frame, will appear jittery in Rviz as there is no interpolation
    def local_display(self, frame):
        object_list = self.objects.obstacles
        for i in range(len(object_list)):
            marker = Marker()
            marker.header = Header()
            marker.header.frame_id = frame

            marker.ns = "Object_NS"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = 0
            marker.color.r = 0.0
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
            marker.scale.z = 0.3

            self.display_pub.publish(marker)



if __name__ == "__main__":
    try:
        ObstacleDetector()
    except rospy.ROSInterruptException:
        pass

