#!/usr/bin/env python3
#coding=utf8

import rospy
import math
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped, PoseStamped


class JoyToMavros:
    def __init__(self):
        topic_twist = "/mavros/setpoint_velocity/cmd_vel"
        self.max_vertical_twist = 1.0
        self.max_horizontal_twist = 1.5
        self.max_angular_twist = 1.0

        self.data_joy = Joy()
        self.current_pose = PoseStamped()
        self.current_yaw = 0.0
        self.twist_to_mavros = TwistStamped()
        
        rospy.Subscriber("/joy", Joy, self.joy_cb)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_cb)
        self.twist_pub = rospy.Publisher(topic_twist, TwistStamped, queue_size=10)

        while not rospy.is_shutdown():
            if len(self.data_joy.axes) != 0:
                data_joy_x0 = self.data_joy.axes[0]  # -1.0 < data_joy.axes[i] < 1.0
                data_joy_y0 = self.data_joy.axes[1]
                data_joy_x1 = self.data_joy.axes[3]
                data_joy_y1 = self.data_joy.axes[4]

                self.twist_to_mavros.header.frame_id = "map"
                vector_twist = (self.max_horizontal_twist * data_joy_y1, self.max_horizontal_twist * data_joy_x1)
                rotate_vector_twist = self.transform_cord(self.current_yaw, vector_twist)
                self.twist_to_mavros.twist.linear.x = rotate_vector_twist[0]
                self.twist_to_mavros.twist.linear.y = rotate_vector_twist[1]
                self.twist_to_mavros.twist.linear.z = self.max_vertical_twist * data_joy_y0

                self.twist_to_mavros.twist.angular.z = self.max_angular_twist * data_joy_x0

                # print(twist_to_mavros)

                self.twist_pub.publish(self.twist_to_mavros)
                # print(data_joy_x0, data_joy_y0, data_joy_x1, data_joy_y1)
            else:
                print("Please touch sticks")


    def joy_cb(self, msg_data):
        self.data_joy = msg_data


    def pose_cb(self, msg_data: PoseStamped):
        self.current_pose = msg_data
        quatenion = (self.current_pose.pose.orientation.x, 
                     self.current_pose.pose.orientation.y, 
                     self.current_pose.pose.orientation.z, 
                     self.current_pose.pose.orientation.w)

        self.current_yaw = euler_from_quaternion(quatenion)[2]
        print(self.current_yaw)
    # TODO: Добавить вращение вестора скорости с учетом рыскания квадрокоптера

    
    # функция преобразования локальных координат в глобальные координаты
    def transform_cord(self, W, cords):
        X = (math.cos(W) * cords[0] - math.sin(W) * cords[1])
        Y = (math.sin(W) * cords[0] + math.cos(W) * cords[1])
        # print (W, X, Y)
        return X, Y


if __name__ == "__main__":
    rospy.init_node('Joy_to_mavros_node')
    JoyToMavros()