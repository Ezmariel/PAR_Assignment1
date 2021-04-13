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
from random import random


class Hazard_search():

    def __init__(self):
        # Cv Bridge used for converting depth images
        self.cvBridge = CvBridge()

        # When a sign is found, this flag gets set
        self.checkDepth = 0

        # ID of the last detected sign
        self.signID = 0

        # creating a new marker object to be published
        self.marker_object = Marker()
        self.marker_object_rr = Marker()

        # creating a listener for the transformations
        self.listener = tf.TransformListener()

        # creating a stamped pose of the marker in robot relative space
        self.marker_pose_rr = geometry_msgs.msg.PoseStamped()

        # creating a list for markers found
        self.marker_list = []

        # Dictionary of markers with a list of poses where they were seen
        self.markerPoses = {}

        # Dictionary of the published poses for markers
        self.publishedMarkers = {}

        # Coords of the middle of a detected sign [x, y]
        self.signMiddle = [-1, -1]

        # distance to a marker
        self.sign_depth = 0

        # When exploration finishes, do some manual moving
        self.posePublisher = rospy.Publisher('/move_base_simple/goal', geometry_msgs.msg.PoseStamped, queue_size=1)

        # Send messages to explore to control, send PAUSE or GO
        self.exploreCommandPublisher = rospy.Publisher('/explore_cmd', String, queue_size=10)

        # Subscribe to depth camera
        rospy.Subscriber("/camera/depth_registered/image_raw", Image, self.processImage)

        # Subscribe to object recognition
        rospy.Subscriber("/objects", Float32MultiArray, self.objectSeen)

        # Subscribe to explore messages
        rospy.Subscriber("/explore_msg", String, self.processExploreMessage)

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
        if len(data.data) > 0:
            # rounding because id stored as float
            self.signID = int(data.data[0])
            if self.signID < 100 and self.signID not in self.marker_list and self.checkDepth == 0:
                rospy.loginfo('marker seen')
                rospy.loginfo("id = " + str(self.signID))
                self.marker_list.append(self.signID)
                self.marker_seen = rospy.Time.now()
                self.savePose(self.signID)
                self.exploreCommandPublisher.publish("PAUSE")

                width = data.data[1]
                height = data.data[2]
                
                qtMatrix = QTransform(data.data[3], data.data[4], data.data[5],
                                    data.data[6], data.data[7], data.data[8], 
                                    data.data[9], data.data[10], data.data[11])
            
                middle = qtMatrix.map(width / 2, height / 2)
                self.signMiddle[0] = int(round(middle[0]))
                self.signMiddle[1] = int(round(middle[1]))

                self.checkDepth = 1

                rospy.loginfo(middle)
                rospy.loginfo(self.signMiddle)

                
            #else:
                #rospy.loginfo("id = Start Marker")

    def processImage(self, data):
        try:
            cvImage = self.cvBridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.loginfo(e)
        
        if self.checkDepth == 1:
            self.checkDepth = 2
            

            self.sign_depth = cvImage[self.signMiddle[1], self.signMiddle[0]]
            rospy.loginfo(self.sign_depth)
            
            if math.isnan(self.sign_depth):
                self.sign_depth = 0.4
                yOffset = 0
                zOffset = 0.2
            else:
                # Calculate y-offset
                radiansPerPixelWidth = 0.00164   # 60deg / 640px = 0.09375deg = 0.00164rad
                if self.signMiddle[0] < 320:
                    theta = (320 - self.signMiddle[0]) * radiansPerPixelWidth
                    yOffset = self.sign_depth * math.sin(theta)
                else:
                    theta = (self.signMiddle[0] - 320) * radiansPerPixelWidth
                    yOffset = -(self.sign_depth * math.sin(theta))

                # rospy.loginfo("theta: " + str(theta) + ", yOffset: " + str(yOffset))

                # Calculate z-offset
                radiansPerPixelHeight = 0.0018   # 49.5deg / 480px = 0.103125deg = 0.0018
                if self.signMiddle[1] < 240:
                    theta = (240 - self.signMiddle[1]) * radiansPerPixelHeight
                    zOffset = self.sign_depth * math.sin(theta)
                else:
                    theta = (self.signMiddle[1] - 240) * radiansPerPixelHeight
                    zOffset = -(self.sign_depth * math.sin(theta))

            self.placeMarker(x_val = self.sign_depth, y_val = yOffset, z_val=zOffset)


    def placeMarker(self, x_val = 0, y_val = 0, z_val = 0):
        rospy.loginfo('transforming marker')
    
        marker_transformed = False

        while not marker_transformed:
            try:
                dest = '/map'
                src = '/base_link'

                # creating a stamped pose of the marker in robot relative space
                self.marker_pose_rr.header.stamp = rospy.Time.now()
                self.marker_pose_rr.header.frame_id = src
                self.marker_pose_rr.pose.position.x = x_val
                self.marker_pose_rr.pose.position.y = y_val
                self.marker_pose_rr.pose.position.z = z_val
                self.marker_pose_rr.pose.orientation.x = 0.0
                self.marker_pose_rr.pose.orientation.y = 0.0
                self.marker_pose_rr.pose.orientation.z = 0.0
                self.marker_pose_rr.pose.orientation.w = 1.0

                # transform the pose to map relative
                rospy.loginfo('transforming')
                self.transpose_mr = self.listener.transformPose(dest, self.marker_pose_rr)
                
                marker_transformed = True
            
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        rospy.loginfo('map relative marker')
        # place marker in map relative
        self.marker_object.header.frame_id = '/map'
        self.marker_object.header.stamp = rospy.Time.now()
        self.marker_object.ns = 'hazard_marker'
        self.marker_object.id = self.signID
        self.marker_object.type = Marker.SPHERE
        self.marker_object.action = Marker.ADD
        self.marker_object.frame_locked = True

        self.marker_object.pose = self.transpose_mr.pose

        self.marker_object.scale.x = 0.2
        self.marker_object.scale.y = 0.2
        self.marker_object.scale.z = 0.2

        self.marker_object.color.r = random()
        self.marker_object.color.g = random()
        self.marker_object.color.b = random()
        self.marker_object.color.a = 1.0

        hp.publish(self.marker_object)
        if (self.marker_object.id not in self.publishedMarkers):
            self.publishedMarkers[self.marker_object.id] = self.marker_object

        self.checkDepth = 0
        self.signID = 0

        self.exploreCommandPublisher.publish("GO")

        rospy.loginfo('marker placed')

    def processExploreMessage(self, data):
        rospy.loginfo("Explore message received: " + data.data)

        if data.data == "DONE":
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
