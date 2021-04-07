#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32MultiArray


class Hazard_search():
    objectId = -1

    marker_object = Marker()

    #def __init__(self):

    def objectSeen(self, data):
        if len(data.data) > 0:
            # rounding because id stored as float
            self.objectId = int(data.data[0])
            if self.objectId < 100:
                rospy.loginfo("id = " + str(self.objectId))


                self.marker_object.header.frame_id = '/odom'
                self.marker_object.header.stamp = rospy.get_rostime()
                self.marker_object.ns = 'hazard_marker'
                self.marker_object.id = self.objectId
                self.marker_object.type = Marker.SPHERE
                self.marker_object.action = Marker.ADD

                self.marker_object.pose.position.x = 1
                self.marker_object.pose.position.y = 1
                self.marker_object.pose.position.z = 1

                self.marker_object.pose.orientation.x = 0.0
                self.marker_object.pose.orientation.y = 0.0
                self.marker_object.pose.orientation.z = 0.0
                self.marker_object.pose.orientation.w = 1.0
                self.marker_object.scale.x = 1.0
                self.marker_object.scale.y = 1.0
                self.marker_object.scale.z = 1.0

                self.marker_object.color.r = 0.0
                self.marker_object.color.g = 0.0
                self.marker_object.color.b = 1.0
                self.marker_object.color.a = 1.0


                hp.publish(self.marker_object)
            else:
                rospy.loginfo("id = Start Marker")

    def execute(self):
	    # Subscribe to the image recognition node
        rospy.Subscriber("/objects", Float32MultiArray, self.objectSeen)
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
