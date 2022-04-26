"""
Программа принимает поправку с РТК через бродкас и отправляет её на GPS приёмник.

PORT по умолчанию 9000
"""

import rclpy
from socket import *
import time
import serial
import sys
from mavros_msgs.msg import RTCM

UDP_PORT = 9000

port = "/dev/ttyACM0"
baud = 115200
rtcm_topic = "/mavros/gps_rtk/send_rtcm"

mav_send = True

if __name__ == '__main__':
    rospy.init_node("RTK_client_node",anonymous=True)
    #
    # # get params
    mav_send = rospy.get_param("~mav_send", mav_send)
    # baud = rospy.get_param("~baud", baud)
    # UDP_PORT = rospy.get_param("~udp_port", UDP_PORT)

    if mav_send:
        mav_pub = rospy.Publisher(rtcm_topic, RTCM, queue_size=10)
    else:
        # check port
        try:
            ser = serial.Serial(
                port=port,
                baudrate=baud,
                parity=serial.PARITY_ODD,
                stopbits=serial.STOPBITS_TWO,
                bytesize=serial.SEVENBITS
            )
            ser.isOpen()
        except:
            print("port not found")
            exit()


    # init server params
    udp_server_sock = socket(AF_INET, SOCK_DGRAM)  # Internet,UDP
    udp_server_sock.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
    udp_server_sock.bind(("", UDP_PORT))


    prev_time = time.time()
    print("port open")
    msgs = RTCM()
    msgs.header.frame_id = "map"
    try:
        while True:
            delay = time.time() - prev_time
            prev_time = time.time()
            # send to serial
            binData, addr = udp_server_sock.recvfrom(1024)  # buffer size is 1024 bytes

            if mav_send:
                msgs.header.stamp = rospy.Time.now()
                msgs.data = binData
                mav_pub.publish(msgs)
            else:
                ser.write(binData)
            print("Send RTCM3 to RTK. Delay: %f" %delay)
    except:
        if not mav_send:
            ser.close()
            print("ERROR: close serial")
