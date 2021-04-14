#!/usr/bin/env python

import rospy
import math
import tf
import tf2_ros
import tf_conversions
import geometry_msgs
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray, String
from cv_bridge import CvBridge, CvBridgeError
from PySide2.QtGui import QTransform
from sensor_msgs.msg import Image
from find_object_2d.msg import ObjectsStamped
from random import random


class Hazard_search():

    def __init__(self):
        # Cv Bridge used for converting depth images
        self.cvBridge = CvBridge()

        # creating a listener for the transformations
        self.listener = tf.TransformListener()

        # creating a list for markers found
        self.marker_list = []

        # Set when explore has finished
        self.returningToOrigin = False

        # When exploration finishes, do some manual moving
        self.posePublisher = rospy.Publisher('/move_base_simple/goal', geometry_msgs.msg.PoseStamped, queue_size=1)

        # Send messages to explore to control, send PAUSE or GO
        self.exploreCommandPublisher = rospy.Publisher('/explore_cmd', String, queue_size=10)

        # Subscribe to object recognition
        rospy.Subscriber("/objectsStamped", ObjectsStamped, self.objectSeen)

    # Save poses, so we can return for more accurate marker mapping
    def savePose(self, markerId):
        dest = '/map'
        src = '/base_link'

        poseFound = False

        if markerId not in self.markerPoses:
            self.markerPoses[markerId] = []
        
        while not poseFound:
            try:
                transformTime = rospy.Time.now()
                self.listener.lookupTransform(dest, src, transformTime)
                self.listener.waitForTransform(dest, src, transformTime, rospy.Duration(0.5))

                robotPose = geometry_msgs.msg.PoseStamped()
                robotPose.header.frame_id = src
                robotPose.header.stamp = transformTime
                pose = self.listener.transformPose(dest, robotPose)
                self.markerPoses[markerId].append(pose)

                rospy.loginfo(self.markerPoses)
            
                poseFound = True

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

    def objectSeen(self, data):
        numSeen = int(len(data.objects.data) / 12)
        
        if numSeen > 0:
            for i in range(numSeen):
                # rounding because id stored as float
                signID = int(data.objects.data[i])
                if signID < 100 and signID not in self.marker_list:
                    rospy.loginfo('marker seen')
                    rospy.loginfo("id = " + str(signID))
                    self.exploreCommandPublisher.publish("PAUSE")
                    self.placeMarker(signID, data.header.stamp)
    

    def placeMarker(self, markerID, timeStamp):
        frameId = "hazard_" + str(markerID)
        
        self.listener.lookupTransform('/map', frameId, timeStamp)

        signPose = geometry_msgs.msg.PoseStamped()
        signPose.header.frame_id = frameId
        signPose.header.stamp = timeStamp
        newMarker = self.listener.transformPose('/map', signPose)

        marker = Marker()
        marker.header.frame_id = '/map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'hazard_marker'
        marker.id = markerID
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.frame_locked = True

        marker.pose = newMarker.pose

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.r = random()
        marker.color.g = random()
        marker.color.b = random()
        marker.color.a = 1.0

        hp.publish(marker)

        self.marker_list.append(markerID)
        self.exploreCommandPublisher.publish("GO")


    def processExploreMessage(self, data):
        rospy.loginfo("Explore message received: " + data.data)

        if data.data == "DONE":
            self.returningToOrigin = True
            self.returnToOrigin()


    def returnToOrigin():
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
        rospy.loginfo("running")

        rospy.spin()


# Short ROS Node method
if __name__ == '__main__':
    try:
        hp = rospy.Publisher('/hazards', Marker, queue_size=1)
        rospy.init_node('hazard_search', anonymous=True)
        
        hs = Hazard_search()
        hs.execute()
    except rospy.ROSInterruptException:
        pass
