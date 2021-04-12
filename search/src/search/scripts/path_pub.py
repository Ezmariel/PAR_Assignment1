#!/usr/bin/env python

import rospy
import math
import tf
import tf2_ros
import tf_conversions
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PublishPath():

    def execute(self):
        # publish the path
        pathPublisher = rospy.Publisher('/path', Path, queue_size=1)

        # creating a listener for the transformations
        transformListener = tf.TransformListener()

        dest = '/map'
        src = '/base_link'

        # array of poses to be published
        path = Path()
        path.header.frame_id = dest

        publishRateCounter = 0
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():

            try:
                transformTime = rospy.Time.now()
                transformListener.lookupTransform(dest, src, transformTime)
                transformListener.waitForTransform(dest, src, transformTime, rospy.Duration(1.0))

                robotPose = PoseStamped()
                robotPose.header.frame_id = src
                robotPose.header.stamp = transformTime
                newPathPoint = transformListener.transformPose(dest, robotPose)
                path.poses.append(newPathPoint)
            
                # Limit the publish rate 
                if publishRateCounter == 5:
                    pathPublisher.publish(path)
                    publishRateCounter = 0
                else:
                    publishRateCounter += 1

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
