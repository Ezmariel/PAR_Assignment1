#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from PySide2.QtGui import QTransform
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

class Depth():
    
    def __init__(self):
        # Cv Bridge used for converting depth images
        self.cvBridge = CvBridge()

        # When a sign is found, this flag gets set
        self.checkDepth = False

        # ID of the last detected sign
        self.signID = 0

        # Coords of the middle of a detected sign [x, y]
        self.signMiddle = [-1, -1]

        # Subscribe to depth camera
        rospy.Subscriber("/camera/depth_registered/image_raw", Image, self.processImage)

        # Subscribe to object recognition
        rospy.Subscriber("/objects", Float32MultiArray, self.objectSeen)

    def objectSeen(self, data):
        if len(data.data) > 0:
            # rounding because id stored as float
            self.signID = int(data.data[0])
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


    def processImage(self, data):
        try:
            cvImage = self.cvBridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.loginfo(e)
        
        if self.checkDepth:
            rospy.loginfo(cvImage[self.signMiddle[0], self.signMiddle[1]])
            self.checkDepth = False

    def execute(self):  

        rospy.spin()

# Short ROS Node method
if __name__ == '__main__':
    try:
        rospy.init_node('depth', anonymous=True)
        
        depth = Depth()
        depth.execute()
    except rospy.ROSInterruptException:
        pass
