# Autonomous-Navigation
# Zack Allen Nicholas Weiland Jordan Domsky Alex Martin

How to use this package:

1 Have ROS kinetic installed: http://wiki.ros.org/kinetic/Installation/Ubuntu
2 Put this repo in your ~/ directory
3 In the workspace's root directory run the following command: 
	bash workspace_setup.bash
  If no errors, the catkin workspace is set up and necessary dependencies are installed
4 To run the simulation use the following commands
  each in a new terminal tab:
	roscore 
        roslaunch ackermann-vehicle-gazebo ackermann-vehicle.launch
        rosrun agc_test control.py
	rosrun agc_test cmd_vel_to_ackermann_msgs.py
5 To issue commands to the simulated robot:
  In the root directory of this workspace there is a bash script
  called test, which lets you talk to the control.py node.
  Run it using:
	 ./test <velocity> <steering angle>

	  
