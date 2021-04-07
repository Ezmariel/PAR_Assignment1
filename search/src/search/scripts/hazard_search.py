#!/usr/bin/env python

import rospy
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
        self.checkDepth = False

        # ID of the last detected sign
        self.signID = 0

        # creating a new marker object to be published
        self.marker_object = Marker()

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
            if self.signID < 100:
                rospy.loginfo("id = " + str(self.signID))

                width = data.data[1]
                height = data.data[2]
                
                qtMatrix = QTransform(data.data[3], data.data[4], data.data[5],
                                    data.data[6], data.data[7], data.data[8], 
                                    data.data[9], data.data[10], data.data[11])
            
                middle = qtMatrix.map(width / 2, height / 2)
                self.signMiddle[0] = int(round(middle[0]))
                self.signMiddle[1] = int(round(middle[1]))
                self.checkDepth = True

                rospy.loginfo(middle)
                rospy.loginfo(self.signMiddle)

                
            else:
                rospy.loginfo("id = Start Marker")

    def processImage(self, data):
        try:
            cvImage = self.cvBridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.loginfo(e)
        
        if self.checkDepth:
            self.sign_depth = cvImage[self.signMiddle[0], self.signMiddle[1]]
            rospy.loginfo(self.sign_depth)
            self.checkDepth = False
            self.placeMarker(x_val = self.sign_depth, signNum = self.signID)

    def placeMarker(self, x_val = 0, y_val = 0, signNum = 0):
        rospy.loginfo('placing marker')
        self.marker_object.header.frame_id = '/map'
        self.marker_object.header.stamp = rospy.get_rostime()
        self.marker_object.ns = 'hazard_marker'
        self.marker_object.id = signNum
        self.marker_object.type = Marker.SPHERE
        self.marker_object.action = Marker.ADD

        self.marker_object.pose.position.x = x_val
        self.marker_object.pose.position.y = y_val
        self.marker_object.pose.position.z = 0.25

        self.marker_object.pose.orientation.x = 0.0
        self.marker_object.pose.orientation.y = 0.0
        self.marker_object.pose.orientation.z = 0.0
        self.marker_object.pose.orientation.w = 1.0
        self.marker_object.scale.x = 1.0
        self.marker_object.scale.y = 1.0
        self.marker_object.scale.z = 1.0

        self.marker_object.color.r = 0
        self.marker_object.color.g = 1
        self.marker_object.color.b = 0
        self.marker_object.color.a = 1.0

        hp.publish(self.marker_object)

        rospy.loginfo('marker placed')

    def execute(self):
        rospy.loginfo("running")

        rospy.spin()


# Short ROS Node method
if __name__ == '__main__':
    try:
        hp = rospy.Publisher('/hazards', Marker, queue_size=10)
        rospy.init_node('hazard_search', anonymous=True)
        
        hs = Hazard_search()
        hs.execute()
    except rospy.ROSInterruptException:
        pass
