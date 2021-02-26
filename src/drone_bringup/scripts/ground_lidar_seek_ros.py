#!/usr/bin/env python
# coding=utf8

"""
Node for get altitude from seek lidar.
"""

import serial
import rospy
import tf
import threading
import time

import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Quaternion

show_plot_flag = False              # get plot

if show_plot_flag:
    import matplotlib.pyplot as plt

# topics
alt_topic = "/drone/alt"
topic_imu = "/mavros/imu/data"

port = "/dev/ttyUSB0"

# setup params
_rate = 80.0        # rate 80 Hz
_rate_lidar = 80.

srez = 20
lidar_offset = 0.0
speed_change = 1.0

use_filter_flag = True

# init values
current_val_lidar = 0.
alt_msg = Float32()
alt_array = list()
offset_filter = 0.0
init_filter = False
rotate = Quaternion()
srez_lidar = list()
ser = None


def filter(current_data, prev_data, current_speed, max_speed):
    """
    Pub alt from lidar
    :param alt:
    :return:
    """
    global offset_filter
    if current_speed < -max_speed:
        print("jump UP")
        offset_filter = prev_data - current_data
        print(offset_filter)
    if current_speed > max_speed:
        print("jump DOWN")
        offset_filter = 0.0     # prev_data - current_data
        print(offset_filter)


def getLidarData():
    """
    Get lidar data from lidar
    :type data: LaserScan
    :return:
    """

    global current_val_lidar, rotate, lidar_offset, srez_lidar, ser, use_filter_flag
    distance = 0.0
    _delay = 1./_rate_lidar
    try:
        while True:
            count = ser.in_waiting
            if count > 8:
                recv = ser.read(9)
                ser.reset_input_buffer()
                if recv[0] == 'Y' and recv[1] == 'Y':  # 0x59 is 'Y'
                    low = int(recv[2].encode('hex'), 16)
                    high = int(recv[3].encode('hex'), 16)
                    distance = (low + high * 256) * 0.01


            # # пересчитываеем проектцию по углам
            euler = tf.transformations.euler_from_quaternion((rotate.x, rotate.y, rotate.z, rotate.w))
            dist_norm = (np.cos(euler[0]) * distance) * np.cos(euler[1])

            current_val_lidar = dist_norm - lidar_offset

            if use_filter_flag:
                srez_lidar.append(current_val_lidar)
                if len(srez_lidar) > srez:
                    del srez_lidar[0]

            time.sleep(_delay)
    except:
        if ser != None:
            ser.close()
        exit()


def callbackRot(data):
    """
    Callback IMU
    :type data: Imu
    :param data: data from tf
    :return:
    """
    global rotate
    rotate = data.orientation


if __name__ == '__main__':

    # Init node
    rospy.init_node('lidar_seek_node', anonymous=True)

    rate = rospy.Rate(_rate)        # set rate

    # init params
    port = rospy.get_param("~port", port)
    use_filter_flag = rospy.get_param("~use_filter", use_filter_flag)
    # Subscriber
    rospy.Subscriber(topic_imu, Imu, callbackRot)

    # Publisher
    alt_pub = rospy.Publisher(alt_topic, Float32, queue_size=10)

    old_time = rospy.get_time()
    prev_lidar_val = 0.0
    prev_baro_val = 0.0

    if show_plot_flag:
        fig = plt.figure()
        ax1 = fig.add_subplot(2, 1, 1)
        ax2 = fig.add_subplot(2, 1, 2)
        lidar_list = list()
        speed_lidar_list = list()
        time_list = list()
        filter_list = list()
        _time = 0.0
        lidar_data = 0.
        speed_lidar = 0.
        filter_data = 0.

    # init serial
    ser = serial.Serial(port, 115200)
    if ser.is_open == False:
        ser.open()

    # get data in external thread
    thread = threading.Thread(target=getLidarData)
    thread.daemon = True
    thread.start()
    # loop
    try:
        while not rospy.is_shutdown():
                # update msgs

                dt = rospy.get_time()-old_time
                old_time = rospy.get_time()


                if use_filter_flag:
                    current_val_lidar

                    # calculate speed of change hight
                    if init_filter:
                        speed_lidar = ((current_val_lidar - prev_lidar_val) / dt)
                    else:
                        init_filter = True
                        speed_lidar = 0.

                    prev_lidar_val = current_val_lidar

                    # get filter
                    if len(srez_lidar) > 0:
                        filter(current_val_lidar, srez_lidar[0], speed_lidar, speed_change)
                    filter_data = current_val_lidar + offset_filter

                    # # reset error
                    if filter_data - current_val_lidar < 0.2:
                        offset_filter = 0.


                    # # publish data
                    alt_msg.data = filter_data
                else:
                    alt_msg.data = current_val_lidar

                alt_pub.publish(alt_msg)

                if show_plot_flag:
                    lidar_list.append(current_val_lidar)
                    speed_lidar_list.append(speed_lidar)
                    time_list.append(_time)
                    filter_list.append(filter_data)

                    if len(lidar_list) > 100:
                        del lidar_list[0]
                        del time_list[0]
                        del speed_lidar_list[0]
                        del filter_list[0]

                    ax1.clear()
                    ax2.clear()

                    ax1.plot(time_list, lidar_list, label='sonar')
                    ax1.plot(time_list, filter_list, label='filter')
                    ax1.set_ylabel('x, m')
                    ax1.set_xlabel('t, s')
                    ax1.legend()
                    ax1.grid()

                    ax2.plot(time_list, speed_lidar_list, label='speed sonar')
                    ax2.set_ylabel('x, m/s')
                    ax2.set_xlabel('t, s')
                    ax2.legend()
                    ax2.grid()

                    plt.pause(1.0 / _rate)
                    _time += dt
                else:
                    rate.sleep()
        ser.close()

    except rospy.ROSInterruptException:
        if ser != None:
            ser.close()
    except:  # Ctrl+C
        if ser != None:
            ser.close()
