#!/usr/bin/env python

import rospy
import math
import tf
import tf2_ros
import tf_conversions
import geometry_msgs
from find_object_2d.msg import ObjectsStamped
from random import random
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class Hazard_search():

    def __init__(self):
        # Listener for the transformations
        self.listener = tf.TransformListener()

        # Markers found so far
        self.marker_list = []

        # Publish the located hazards
        self.hazardPublisher = rospy.Publisher('/hazards', Marker, queue_size=1)

        # Send messages to explore to control; PAUSE or GO
        self.exploreCommandPublisher = rospy.Publisher('/explore_cmd', String, queue_size=10)

        # Subscribe to object recognition
        rospy.Subscriber("/objectsStamped", ObjectsStamped, self.objectSeen)

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

        # Get the pose of the located hazard        
        self.listener.lookupTransform('/map', frameId, timeStamp)

        signPose = geometry_msgs.msg.PoseStamped()
        signPose.header.frame_id = frameId
        signPose.header.stamp = timeStamp
        newMarker = self.listener.transformPose('/map', signPose)

        # Create a marker, which will be published to /hazards
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

        # Publish the marker to /hazards
        self.hazardPublisher.publish(marker)

        self.marker_list.append(markerID)
        self.exploreCommandPublisher.publish("GO")


    def execute(self):
        rospy.loginfo("running")

        rospy.spin()


# Short ROS Node method
if __name__ == '__main__':
    try:
        rospy.init_node('hazard_search', anonymous=True)
        
        hs = Hazard_search()
        hs.execute()
    except rospy.ROSInterruptException:
        pass
