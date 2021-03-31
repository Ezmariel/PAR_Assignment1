#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray


class Hazard_search():
    objectId = -1

    #def __init__(self):

    def objectSeen(self, data):
        if len(data.data) > 0:
            # rounding because id stored as float
            self.objectId = int(data.data[0])
            rospy.loginfo("id = " + str(self.objectId))

    def execute(self):
	    # Subscribe to the image recognition node
        rospy.Subscriber("/objects", Float32MultiArray, self.objectSeen)
        rospy.loginfo("running")

        rospy.spin()

# Short ROS Node method
if __name__ == '__main__':
    try:
        rospy.Publisher('hazards', Marker, queue_size=10)
        rospy.init_node('rosbot_waypoint', anonymous=True)
        
        hs = Hazard_search()
        hs.execute()
    except rospy.ROSInterruptException:
        pass
