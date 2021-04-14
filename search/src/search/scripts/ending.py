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

        # spin rotation
        self.spinRot = 0

        # have we spun?
        self.spunOnce = False
        
        # have we arrived at the goal?
        self.atGoal = False

        # creating a pose for spinning
        self.spinPose = geometry_msgs.msg.PoseStamped()
        self.poseSaved = False

    # Save poses, so we can return for more accurate marker mapping
    def savePose(self):
        dest = '/map'
        src = '/base_link'

        poseFound = False
        
        while not poseFound:
            try:
                transformTime = rospy.Time.now()
                self.listener.lookupTransform(dest, src, transformTime)
                self.listener.waitForTransform(dest, src, transformTime, rospy.Duration(0.5))

                robotPose = geometry_msgs.msg.PoseStamped()
                robotPose.header.frame_id = src
                robotPose.header.stamp = transformTime
                self.spinPose = self.listener.transformPose(dest, robotPose)
            
                poseFound = True

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
        
        self.poseSaved = True

    def processExploreMessage(self, data):
        rospy.loginfo("Explore message received: " + data.data)
        rospy.loginfo("Beginning end segment")

        self.savePose()
        rospy.loginfo("Beginning end segment")
        self.exploreDone = True

    def pauseListener(self, data):
        if self.exploreDone == True:
            if data.data == "PAUSE":
                self.paused = True
                rospy.loginfo("Pausing in end segment")
                if self.spinRot == 0:
                    self.atGoal = False
                elif self.spinRot > 0:
                    self.spinRot = self.spinRot - 1
                    if self.spinRot == 6 and self.spunOnce == True:
                        self.spunOnce = False

            elif data.data == "GO":
                self.paused = False
                rospy.loginfo("continuing in end segment")
        else:
            pass
    
    def spinAround(self):
        if self.paused:
            pass
        else:
            if self.poseSaved == False:
                self.savePose()

            rospy.loginfo("Spinning around")
            target_pose = geometry_msgs.msg.PoseStamped()
            target_pose.header.frame_id = 'map'
            target_pose.header.stamp = rospy.Time.now()
            target_pose.pose = self.spinPose.pose
            target_pose.pose.orientation.z = (60 * self.spinRot) * 3.14 / 180
            target_pose.pose.orientation.w = 1
            self.spunOnce = self.spinRot + 1
            if self.spunOnce == False and self.spinRot > 5:
                self.spunOnce = True
            self.posePublisher.publish(target_pose)

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
            self.atGoal = True

    def execute(self):
        rate = rospy.Rate(5.0)

        missionComplete = False

        while not missionComplete:
            if self.exploreDone == True:
                if self.spunOnce == False:
                    self.spinAround()
                elif self.atGoal == False:
                    self.poseSaved = False
                    self.spinRot = 0
                    self.goToGoal()
                elif self.spinRot < 6:
                    self.spinAround()
                else:
                    rospy.loginfo("MISSION COMPLETE!")
                    missionComplete = True

            rate.sleep()


        rospy.spin()


# Short ROS Node method
if __name__ == '__main__':
    try:
        rospy.init_node('ending', anonymous=True)
        
        ending = Ending()
        ending.execute()
    except rospy.ROSInterruptException:
        pass