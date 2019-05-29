# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "navigation_msgs: 5 messages, 0 services")

set(MSG_I_FLAGS "-Inavigation_msgs:/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(navigation_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/LatLongPoint.msg" NAME_WE)
add_custom_target(_navigation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "navigation_msgs" "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/LatLongPoint.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/EmergencyStop.msg" NAME_WE)
add_custom_target(_navigation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "navigation_msgs" "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/EmergencyStop.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/FErequest.msg" NAME_WE)
add_custom_target(_navigation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "navigation_msgs" "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/FErequest.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/VelAngle.msg" NAME_WE)
add_custom_target(_navigation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "navigation_msgs" "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/VelAngle.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/WaypointsArray.msg" NAME_WE)
add_custom_target(_navigation_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "navigation_msgs" "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/WaypointsArray.msg" "sensor_msgs/NavSatStatus:std_msgs/Header:sensor_msgs/NavSatFix"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/LatLongPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation_msgs
)
_generate_msg_cpp(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/EmergencyStop.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation_msgs
)
_generate_msg_cpp(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/FErequest.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation_msgs
)
_generate_msg_cpp(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/VelAngle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation_msgs
)
_generate_msg_cpp(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/WaypointsArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/NavSatStatus.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/NavSatFix.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(navigation_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(navigation_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(navigation_msgs_generate_messages navigation_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/LatLongPoint.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_cpp _navigation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/EmergencyStop.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_cpp _navigation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/FErequest.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_cpp _navigation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/VelAngle.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_cpp _navigation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/WaypointsArray.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_cpp _navigation_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(navigation_msgs_gencpp)
add_dependencies(navigation_msgs_gencpp navigation_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS navigation_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/LatLongPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/navigation_msgs
)
_generate_msg_eus(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/EmergencyStop.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/navigation_msgs
)
_generate_msg_eus(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/FErequest.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/navigation_msgs
)
_generate_msg_eus(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/VelAngle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/navigation_msgs
)
_generate_msg_eus(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/WaypointsArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/NavSatStatus.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/NavSatFix.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/navigation_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(navigation_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/navigation_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(navigation_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(navigation_msgs_generate_messages navigation_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/LatLongPoint.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_eus _navigation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/EmergencyStop.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_eus _navigation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/FErequest.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_eus _navigation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/VelAngle.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_eus _navigation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/WaypointsArray.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_eus _navigation_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(navigation_msgs_geneus)
add_dependencies(navigation_msgs_geneus navigation_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS navigation_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/LatLongPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navigation_msgs
)
_generate_msg_lisp(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/EmergencyStop.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navigation_msgs
)
_generate_msg_lisp(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/FErequest.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navigation_msgs
)
_generate_msg_lisp(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/VelAngle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navigation_msgs
)
_generate_msg_lisp(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/WaypointsArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/NavSatStatus.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/NavSatFix.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navigation_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(navigation_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navigation_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(navigation_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(navigation_msgs_generate_messages navigation_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/LatLongPoint.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_lisp _navigation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/EmergencyStop.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_lisp _navigation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/FErequest.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_lisp _navigation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/VelAngle.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_lisp _navigation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/WaypointsArray.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_lisp _navigation_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(navigation_msgs_genlisp)
add_dependencies(navigation_msgs_genlisp navigation_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS navigation_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/LatLongPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/navigation_msgs
)
_generate_msg_nodejs(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/EmergencyStop.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/navigation_msgs
)
_generate_msg_nodejs(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/FErequest.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/navigation_msgs
)
_generate_msg_nodejs(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/VelAngle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/navigation_msgs
)
_generate_msg_nodejs(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/WaypointsArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/NavSatStatus.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/NavSatFix.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/navigation_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(navigation_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/navigation_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(navigation_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(navigation_msgs_generate_messages navigation_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/LatLongPoint.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_nodejs _navigation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/EmergencyStop.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_nodejs _navigation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/FErequest.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_nodejs _navigation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/VelAngle.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_nodejs _navigation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/WaypointsArray.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_nodejs _navigation_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(navigation_msgs_gennodejs)
add_dependencies(navigation_msgs_gennodejs navigation_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS navigation_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/LatLongPoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation_msgs
)
_generate_msg_py(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/EmergencyStop.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation_msgs
)
_generate_msg_py(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/FErequest.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation_msgs
)
_generate_msg_py(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/VelAngle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation_msgs
)
_generate_msg_py(navigation_msgs
  "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/WaypointsArray.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/NavSatStatus.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/NavSatFix.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(navigation_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(navigation_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(navigation_msgs_generate_messages navigation_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/LatLongPoint.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_py _navigation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/EmergencyStop.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_py _navigation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/FErequest.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_py _navigation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/VelAngle.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_py _navigation_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/browermb/jmuautonomous/autonomous-navigation/catkin_ws/src/navigation_msgs/msg/WaypointsArray.msg" NAME_WE)
add_dependencies(navigation_msgs_generate_messages_py _navigation_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(navigation_msgs_genpy)
add_dependencies(navigation_msgs_genpy navigation_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS navigation_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/navigation_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(navigation_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(navigation_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/navigation_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/navigation_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(navigation_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(navigation_msgs_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navigation_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/navigation_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(navigation_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(navigation_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/navigation_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/navigation_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(navigation_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(navigation_msgs_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/navigation_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(navigation_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(navigation_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
