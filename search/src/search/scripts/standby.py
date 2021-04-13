#!/usr/bin/env python

import os
import rospy
from std_msgs.msg import Float32MultiArray

class Standby():

    def __init__(self):
        # flag indicating whether or not the start marker was seen
        self.seen = False

        # Subscribe to object recognition
        self.listener = rospy.Subscriber("/objects", Float32MultiArray, self.objectSeen)

    def objectSeen(self, data):
        if len(data.data) > 0:
            # rounding because id stored as float
            signID = int(data.data[0])
            if signID == 100:
                rospy.loginfo('start marker seen')
                os.system("roslaunch search all.launch")
                self.listener.unregister()
                # self.seen = True

    def execute(self):
        rospy.spin()

        # rate = rospy.Rate(10)
        # while not self.seen:
        #     rate.sleep()


# Short ROS Node method
if __name__ == '__main__':
    try:
        rospy.init_node('standby', anonymous=True)
        
        standby = Standby()
        standby.execute()
    except rospy.ROSInterruptException:
        pass
