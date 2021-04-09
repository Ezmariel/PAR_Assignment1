#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path


class Path_pub():

    def __init__(self):
        pass

    # subscribe to waypoint creating topic
    # publish waypoints to /path

    def execute(self):
        rospy.loginfo("plotting path")

        rospy.spin()    

# Short ROS Node method
if __name__ == '__main__':
    try:
        wp = rospy.Publisher('/path', Path, queue_size=1)
        rospy.init_node('path_pub', anonymous=True)
        
        pp = Path_pub()
        pp.execute()
    except rospy.ROSInterruptException:
        pass
