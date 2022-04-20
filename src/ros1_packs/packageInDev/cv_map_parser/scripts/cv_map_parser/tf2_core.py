#!/usr/bin/env python

import tf2_ros
import geometry_msgs.msg
import rospy


if __name__ == '__main__':
    rospy.init_node('tf2_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('base_link', 'odom', rospy.Time())
            print(trans)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue