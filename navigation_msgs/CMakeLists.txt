cmake_minimum_required(VERSION 2.8.3)
project(navigation_msgs)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)
add_message_files(
    FILES
    FErequest.msg
    LatLongPoint.msg
    WaypointsArray.msg
    VelAngle.msg
    Stop.msg
    SampleData.msg
    Landmarks.msg
    GoalWaypoint.msg
    LocalPointsArray.msg
    VehicleState.msg
	Obstacle.msg
	ObstacleArray.msg
    LatLongArray.msg
)
 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
catkin_package()

