#!/usr/bin/env python
import rospy

import math
import tf2_ros
import tf2_geometry_msgs
from aruco_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import tf

if __name__ == '__main__':
    rospy.init_node('tf2_test_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    pose = PoseStamped()

    pose.header.seq = 0
    pose.header.stamp.secs = 463
    pose.header.stamp.nsecs = 502000000
    pose.header.frame_id = "torso"

    pose.pose.position.x = 0.405374838339
    pose.pose.position.y = -0.0825702495466
    pose.pose.position.z = 0.713030977173

    pose.pose.orientation.x = -0.47721994082
    pose.pose.orientation.y = 0.521688984515
    pose.pose.orientation.z = 0.523046525394
    pose.pose.orientation.w = 0.475945442034


    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('head_camera', 'torso', rospy.Time())
            print(trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        transformed = tf2_geometry_msgs.do_transform_pose(pose, trans)
        print(transformed)

        # msg = geometry_msgs.msg.Twist()
        #
        # msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        # msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
        #
        # turtle_vel.publish(msg)

        rate.sleep()
