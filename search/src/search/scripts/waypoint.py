#!/usr/bin/env python

import rospy

import actionlib
from geometry_msgs.msg import (
    PoseStamped,
    Twist
)
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Float32MultiArray
from tf import TransformListener

class Waypoint():
    objectId = 0

    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.map_frame = rospy.get_param("~map_frame", 'map')
        self.timeout = rospy.get_param("~timeout", 30)

    def objectSeen(self, data):
        if len(data.data) > 0:
            # rounding because id stored as float
            self.objectId = int(data.data[0])

    def execute(self):
	    # Subscribe to the image recognition node
        rospy.Subscriber("/objects", Float32MultiArray, self.objectSeen)

        # Create waypoints
        waypoint_x = 1
        waypoint_y = 0

        # Ensure move_base action client server is available
        rospy.loginfo('Connecting to move_base...')
        timer = self.client.wait_for_server(rospy.Duration(self.timeout))
        if not timer:
            rospy.logerr("Could not connect to move base server, terminating")
            return
        rospy.loginfo('Connected to move_base.')

        start = False
        while not start:
	        # wait for starting marker
            if self.objectId == 100:
                start = True

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.pose.position.x = waypoint_x
        goal.target_pose.pose.position.y = waypoint_y
        goal.target_pose.pose.orientation.w = 1
        rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %
                (goal.target_pose.pose.position.x, goal.target_pose.pose.position.y))
        rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")

        # Send goal
        self.client.send_goal(goal)

        # Iterate through waypoints
        done = False
        while not done:
            # Wait for result
            timer = self.client.wait_for_result(rospy.Duration(self.timeout))
            if not timer:
                rospy.loginfo("Complete goal - may not have arrived - may be blocked")
                rospy.loginfo("\tContinuing...")
            else :
                rospy.logwarn("Wait for move_base complete timed-out")
                state = self.client.get_state()
                rospy.logwarn("Current move_base state:" + str(state))
                rospy.logwarn("\tContinuing to next waypoint...")
                self.client.cancel_goal()
                done = True
                
        # Execute is complete
        return 'completed'

# Short ROS Node method
if __name__ == '__main__':
    try:
        rospy.init_node('rosbot_waypoint', anonymous=True)
        
        wp = Waypoint()
        wp.execute()
    except rospy.ROSInterruptException:
        pass
