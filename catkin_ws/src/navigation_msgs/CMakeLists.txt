cmake_minimum_required(VERSION 2.8.3)
project(navigation_msgs)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)
add_message_files(
    FILES
    fe_request.msg
    lat_long_point.msg
    list_of_goals.msg
)
 generate_messages(
   DEPENDENCIES
   std_msgs
 )
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
catkin_package()

