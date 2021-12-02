#!/usr/bin/env python
#coding=utf8

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped

topic_twist = "/mavros/setpoint_velocity/cmd_vel"
max_twist_vertical = 1.0
max_twist_horizontal = 1.5
max_angular_twist = 1.0

data_joy = Joy()
twist_to_mavros = TwistStamped()

def joy_cb(msg_data):
    global data_joy
    data_joy = msg_data


def main():
    rospy.init_node('Joy_to_mavros_node')
    rospy.Subscriber("/joy", Joy, joy_cb)
    twist_pub = rospy.Publisher(topic_twist, TwistStamped, queue_size=10)

    while not rospy.is_shutdown():
        if len(data_joy.axes) != 0:
            data_joy_x0 = data_joy.axes[0]  # -1.0 < data_joy.axes[i] < 1.0
            data_joy_y0 = data_joy.axes[1]
            data_joy_x1 = data_joy.axes[3]
            data_joy_y1 = data_joy.axes[4]

            twist_to_mavros.twist.linear.x = max_twist_horizontal * data_joy_y1
            twist_to_mavros.twist.linear.y = max_twist_horizontal * data_joy_x1

            twist_to_mavros.twist.linear.z = max_twist_vertical * data_joy_y0

            twist_to_mavros.twist.angular.z = max_angular_twist * data_joy_x0

            # print(twist_to_mavros)

            twist_pub.publish(twist_to_mavros)
            print(data_joy_x0, data_joy_y0, data_joy_x1, data_joy_y1)

        else:
            print("Please touch sticks")

if __name__ == "__main__":
    main()