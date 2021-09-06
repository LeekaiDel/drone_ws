#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Init IMU data from FCU.
This node necessary then ise SLAM position in th PX4

The node send fake position (map --> base_link) t(sec.) after exit.
"""

import rospy
import tf
import time
parent_link = "map_slam"
child_link = "base_link"

tf_br = tf.TransformBroadcaster()

time_delay = 3.0



if __name__ == '__main__':
    rospy.init_node('init_fcu_state_node', anonymous=True)
    rate = rospy.Rate(50)
    print("init state: start")

    time_delay = rospy.get_param("delay", time_delay)

    timer = 0.0
    old_time = time.time()

    while not rospy.is_shutdown():
            dt = time.time() - old_time
            timer += dt

            tf_br.sendTransform((0.0, 0.0, 0.3), (0.0, 0.0, 0.0, 1.0),
                                    rospy.Time.now(), child_link, parent_link)

            old_time = time.time()
            rate.sleep()
            print(timer)
            if timer >= time_delay:
                break
    print("init state: finish")