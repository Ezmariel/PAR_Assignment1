#!/usr/bin/env python

import rospy
import math
import tf
import tf2_ros
import tf_conversions
import geometry_msgs
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray
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

        # Coords of the middle of a detected sign [x, y]
        self.signMiddle = [-1, -1]

        # distance to a marker
        self.sign_depth = 0

        # Subscribe to depth camera
        rospy.Subscriber("/camera/depth_registered/image_raw", Image, self.processImage)

        # Subscribe to object recognition
        rospy.Subscriber("/objects", Float32MultiArray, self.objectSeen)


    def objectSeen(self, data):
        if len(data.data) > 0:
            # rounding because id stored as float
            self.signID = int(data.data[0])
            if self.signID < 100 and self.signID not in self.marker_list and self.checkDepth == 0:
                rospy.loginfo('marker seen')
                rospy.loginfo("id = " + str(self.signID))
                self.marker_list.append(self.signID)
                self.marker_seen = rospy.Time.now()

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

        self.checkDepth = 0
        self.signID = 0

        rospy.loginfo('marker placed')

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
