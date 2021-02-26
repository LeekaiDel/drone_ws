#!/usr/bin/env python
# coding=utf8

"""
Node for get altitude from ultrasonic.
"""



#Libraries
import RPi.GPIO as GPIO
import time
import math


import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Range, Imu
from geometry_msgs.msg import Quaternion
import tf

# topics
alt_topic = "/drone/alt"
topic_imu = "/mavros/imu/data"
mav_topic = "/mavros/distance_sensor/sonar_1_sub"

max_alt = 4.0

#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BOARD) 
#set GPIO Pins
GPIO_TRIGGER = 26 # 24
GPIO_ECHO = 23      # 10
 
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)

rotate = Quaternion()


def distance():
    global rotate
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
    
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
        if StartTime - StopTime > 1.0:
            return None
        
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
        if StopTime - StartTime > 0.05:
            return max_alt

    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = TimeElapsed * 175.5
    
    # solve rotate normal
    euler = tf.transformations.euler_from_quaternion((rotate.x, rotate.y, rotate.z, rotate.w))
    dist_norm = (math.cos(euler[0]) * distance) * math.cos(euler[1])
    return dist_norm

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
    print ("init sonar")

     # Init node
    rospy.init_node('ultrasonic_node', anonymous=True)

    rate = rospy.Rate(30.0)        # set rate

    # Subscriber
    # rospy.Subscriber(topic_imu, Imu, callbackRot)
    # Publisher
    alt_pub = rospy.Publisher(alt_topic, Float32, queue_size=10)
    mav_pub = rospy.Publisher(mav_topic, Range, queue_size=10)

    range = Range()

    range.field_of_view = math.radians(15)
    range.min_range = 0.04
    range.max_range = 4.0
    range.radiation_type = Range.ULTRASOUND
    range.header.frame_id = 'base_link'

    try:
        while not rospy.is_shutdown():
            dist = distance()
            if dist == None:
                rate.sleep()
                continue
            alt_pub.publish(dist)
            range.range = dist
            range.header.stamp = rospy.Time.now()
            mav_pub.publish(range)
            rate.sleep()
 
        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
    except rospy.ROSInterruptException:
        print("Measurement stopped by ROS")
        GPIO.cleanup()
