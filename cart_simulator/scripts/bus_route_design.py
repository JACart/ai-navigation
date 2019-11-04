class bus_route_design():
    def __init__(self):
        rospy.init_node('bus_route_design')

        self.waypoints_pub = rospy.Publish('/global_path', LocalPointsArray, queue_size=10)
        self.waypoints_sub = rospy.Subscriber('/local_points', LocalPointsArray, self.waypoints_callback, queue_size=10)
    
    def waypoints_callback(self, msg):
        # Array of points that have the coordinates of each waypoints
        self.waypoint_list = msg.localpoints
        

    def find_path(self, start, goal):
        # start and goal are indexes with in list of waypoints
	    self.path_to_goal = [] # array of indexs
	    path_found = False
	    index = start

	    while (not path_found):
		    if index == goal: # reach the goal, then end.
			    # self.path_to_goal.append(goal)
                self.path_to_goal.append(self.waypoint_list[goal])
			    path_found = True
			    continue
		    elif index == len(self.waypoint_list) - 1: # reach end of bus route
			    self.path_to_goal.append(self.waypoint_list[index])
			    index = 0 # go back to the beginning
			    continue
		    elif index == 20: # Intersection go left or right.
			    self.path_to_goal.append(self.waypoint_list[index])
			    if goal > 4 and goal < 20:
				    index = 5
			    else:
				    index = 12
			    continue
		    self.path_to_goal.append(self.waypoint_list[index]) # just following bus route numerically
		    index += 1
        
        # Make and Publish msg to WaypointsArray
        msg = LocalPointsArray
        msg.localpoints = self.path_to_goal
        self.waypoint_pub.publish(msg)
		
	
if __name__ == '__main__':
	try:
	    bus_route_design()
    except rospy.ROSInterruptException:
	    pass
