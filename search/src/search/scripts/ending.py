#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import tf_conversions
import geometry_msgs
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray, String


class Ending():

    def __init__(self):

        # creating a listener for the transformations
        self.listener = tf.TransformListener()

        # When exploration finishes, do some manual moving
        self.posePublisher = rospy.Publisher('/move_base_simple/goal', geometry_msgs.msg.PoseStamped, queue_size=1)

        # Subscribe to explore messages
        rospy.Subscriber("/explore_msg", String, self.processExploreMessage)

        # Subscribe to pause messages
        rospy.Subscriber("/explore_cmd", String, self.pauseListener)

        # have we finished exploring?
        self.exploreDone = False

        # are we paused?
        self.paused = False

        

    def processExploreMessage(self, data):
        rospy.loginfo("Explore message received: " + data.data)
        rospy.loginfo("Beginning end segment")
        self.goToGoal()

    def pauseListener(self, data):
        if self.exploreDone == True:
            if data.data == "PAUSE":
                self.paused = True
                rospy.loginfo("Pausing in end segment")

            elif data.data == "GO":
                self.paused = False
                rospy.loginfo("continuing in end segment")
                self.goToGoal
        else:
            pass
    

    def goToGoal(self):
        if self.paused:
            pass
        else:
            rospy.loginfo("Attempting to return to origin")
            target_pose = geometry_msgs.msg.PoseStamped()
            target_pose.header.frame_id = 'map'
            target_pose.header.stamp = rospy.Time.now()
            target_pose.pose.position.x = 0
            target_pose.pose.position.y = 0
            target_pose.pose.position.z = 0
            target_pose.pose.orientation.x = 0
            target_pose.pose.orientation.y = 0
            target_pose.pose.orientation.z = 0
            target_pose.pose.orientation.w = 1
            self.posePublisher.publish(target_pose)

    def execute(self):

        rospy.spin()


# Short ROS Node method
if __name__ == '__main__':
    try:
        rospy.init_node('ending', anonymous=True)
        
        ending = Ending()
        ending.execute()
    except rospy.ROSInterruptException:
        pass