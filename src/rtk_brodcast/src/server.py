"""
Программа для трансляции поправки с BASE RTK в топик ROS.
Данные принимаются через:
1. UART
"""

from socket import *
import time
import serial
from mavros_msgs.msg import RTCM

port = "/dev/ttyACM0"
baud = 115200

if __name__ == '__main__':
    try:
        ser = serial.Serial(
                port=port,
                baudrate=baud,
                parity=serial.PARITY_ODD,
                stopbits=serial.STOPBITS_TWO,
                bytesize=serial.SEVENBITS)
        ser.isOpen()
    except:
        print("port not found")
        exit()
        
    correction_msg = RTCM()
    try:
        while True:

            delay = time.time() - prev_time
            prev_time = time.time()
       
            # get data from serial
            out = ''
            while ser.inWaiting() > 0:
                out += ser.read(1)
            if out != '':
                print("Serial: Get RTK RTCM3. Delay: %f" % (delay))
                correction_msg.data = out
                print(correction_msg)
                # send_brodcast_msgs(out)
    except:
        print("ERROR: close serial")
