cmake_minimum_required(VERSION 2.8.3)
project(navigation)
find_package(catkin REQUIRED)
find_package(message_generation)
<<<<<<< HEAD
add_message_files(DIRECTORY ../navigation_msgs/msg/ FILES FErequest.msg VelAngle.msg LatLongPoint.msg WaypointsArray.msg)
=======
add_message_files(DIRECTORY ../../navigation_msgs/msg/ FILES FErequest.msg VelAngle.msg LatLongPoint.msg WaypointsArray.msg)
>>>>>>> 095bd448bd72a394aceaa418d7a2d9bed057d9ea
catkin_metapackage()
catkin_package(CATKIN_DEPENDS message_runtime)

