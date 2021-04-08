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

        # creating a listener for the transformations
        self.listener = tf.TransformListener()

        # creating a stamped pose of the marker in robot relative space
        self.marker_pose_rr = geometry_msgs.msg.PoseStamped()

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
            if self.signID < 100 and self.checkDepth == 0:
                rospy.loginfo("id = " + str(self.signID))

                width = data.data[1]
                height = data.data[2]
                
                qtMatrix = QTransform(data.data[3], data.data[4], data.data[5],
                                    data.data[6], data.data[7], data.data[8], 
                                    data.data[9], data.data[10], data.data[11])
            
                middle = qtMatrix.map(width / 2, height / 2)
                self.signMiddle[0] = int(round(middle[0]))
                self.signMiddle[1] = int(round(middle[1]))

                self.checkDepth = self.checkDepth + 1

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
            self.sign_depth = cvImage[self.signMiddle[0], self.signMiddle[1]]
            rospy.loginfo(self.sign_depth)
            #self.checkDepth = False
            if math.isnan(self.sign_depth):
                self.sign_depth = 1
            self.placeMarker(x_val = self.sign_depth)

    def placeMarker(self, x_val = 0, y_val = 0):
        rospy.loginfo('placing marker')
        rospy.loginfo('POINT 1**************************************')

        marker_transformed = False

        while not marker_transformed:
            try:
                dest = '/map'
                src = '/camera_rgb_optical_frame'
                (trans, rot) = self.listener.lookupTransform(dest, src, rospy.Time.now() - rospy.Duration(0.1))
                print("Transform: " + src + " -> " + dest + ": Translation: (" + str(trans) + "), Rotation: (" + str(rot) + ")" )

                # creating a stamped pose of the marker in robot relative space
                self.marker_pose_rr.header.stamp = rospy.Time.now()
                self.marker_pose_rr.header.frame_id = src
                self.marker_pose_rr.pose.position.x = x_val
                self.marker_pose_rr.pose.position.y = y_val
                self.marker_pose_rr.pose.position.z = 0.0
                self.marker_pose_rr.pose.orientation.x = 0.0
                self.marker_pose_rr.pose.orientation.y = 0.0
                self.marker_pose_rr.pose.orientation.z = 0.0
                self.marker_pose_rr.pose.orientation.w = 1.0

                #pstamped = geometry_msgs.msg.PoseStamped()
                #pstamped.header.stamp = rospy.Time.now() - rospy.Duration(0.1)
                #pstamped.header.frame_id = src
                #pstamped.pose.position.x = 1.0
                #pstamped.pose.position.y = 0.0
                #pstamped.pose.position.z = 0.0
                #pstamped.pose.orientation.x = 0.0
                #pstamped.pose.orientation.y = 0.0
                #pstamped.pose.orientation.z = 0.0
                #pstamped.pose.orientation.w = 1.0

                rospy.loginfo('POINT 2**************************************')

                # transform the pose to map relative
                self.transpose_mr = self.listener.transformPose(dest, self.marker_pose_rr)
                print("\ttransformed: \n" + str(self.transpose_mr.pose))

                marker_transformed = True
            
                rospy.loginfo('POINT 3**************************************')

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                #print("could not load transform: " + src + " -> " + dest)
                pass


        self.marker_object.header.frame_id = '/map'
        self.marker_object.header.stamp = rospy.Time.now()
        self.marker_object.ns = 'hazard_marker'
        self.marker_object.id = self.signID
        self.marker_object.type = Marker.SPHERE
        self.marker_object.action = Marker.ADD

        rospy.loginfo('POINT 4**************************************')

        self.marker_object.pose = self.transpose_mr.pose

        rospy.loginfo('POINT 5**************************************')

        #self.marker_object.pose.position.x = x_val
        #self.marker_object.pose.position.y = y_val
        #self.marker_object.pose.position.z = 0.25
        #self.marker_object.pose.orientation.x = 0.0
        #self.marker_object.pose.orientation.y = 0.0
        #self.marker_object.pose.orientation.z = 0.0
        #self.marker_object.pose.orientation.w = 1.0

        self.marker_object.scale.x = 0.2
        self.marker_object.scale.y = 0.2
        self.marker_object.scale.z = 0.2

        self.marker_object.color.r = (self.signID*11)%3
        self.marker_object.color.g = (self.signID*11)%5
        self.marker_object.color.b = (self.signID*11)%7
        self.marker_object.color.a = 1.0

        rospy.loginfo('POINT 6**************************************')

        hp.publish(self.marker_object)

        rospy.loginfo('POINT 7**************************************')

        rospy.loginfo('marker placed')

        rospy.loginfo('POINT 8**************************************')

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
