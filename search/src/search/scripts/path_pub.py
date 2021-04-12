#!/usr/bin/env python

import rospy
import math
import tf
import tf2_ros
import tf_conversions
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PublishPath():

    def __init__(self):
        self.pathPublisher = rospy.Publisher('/path', Path, queue_size=10)

        # creating a listener for the transformations
        self.transformListener = tf.TransformListener()

    def execute(self):
        rospy.loginfo("running")

        dest = '/map'
        src = '/base_link'

        # array of poses to be published
        path = Path()
        path.header.frame_id = dest

        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():

            try:
                transformTime = rospy.Time.now() #- rospy.Duration(0.1)
                self.transformListener.lookupTransform(dest, src, transformTime)
                # print("Transform: " + src + " -> " + dest + ": Translation: (" + str(trans) + "), Rotation: (" + str(rot) + ")" )
                self.transformListener.waitForTransform(dest, src, transformTime, rospy.Duration(5.0))

                robotPose = PoseStamped()
                robotPose.header.frame_id = src
                robotPose.header.stamp = transformTime
                newPathPoint = self.transformListener.transformPose(dest, robotPose)
                path.poses.append(newPathPoint)
            
                self.pathPublisher.publish(path)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

            rate.sleep()

        rospy.spin()


# Short ROS Node method
if __name__ == '__main__':
    try:
        rospy.init_node('publish_path', anonymous=True)
        
        publishPath = PublishPath()
        publishPath.execute()
    except rospy.ROSInterruptException:
        pass
