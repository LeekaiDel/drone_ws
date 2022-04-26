"""
Программа для трансляции поправки с BASE RTK в топик ROS.
Данные принимаются через:
1. UART
"""


import rclpy
# from rclpy import Node
from socket import *
import time
import serial
from mavros_msgs.msg import RTCM


class EmlidCorrectionTranslator(rclpy.Node):
    def __init__(self):
        super().__init__("emlid_correction_translator_node")

        self.port = "/dev/ttyACM1"
        self.baud = 115200

        self.correction_msg = RTCM()

        # Инициализируем publisher для отправки в topic
        self.correction_pub = self.create_publisher(RTCM, "/correction_rtcm", 10)

        # Подключаемся по serial-порту
        try:
            self.ser = serial.Serial(
                    port=self.port,
                    baudrate=self.baud,
                    parity=serial.PARITY_ODD,
                    stopbits=serial.STOPBITS_TWO,
                    bytesize=serial.SEVENBITS)
            self.ser.isOpen()
        except:
            print("port not found")
            exit()


        # Основной цикл
        try:
            self.prev_time = 0.0
            while True:
                delay = time.time() - self.prev_time
                self.prev_time = time.time()

                self.prev_time = time.time()
                # out = str()
                # get data from serial
                while self.ser.inWaiting() > 0:
                    out = self.ser.read(self.ser.in_waiting)
                if out is not None:
                    print("Serial: Get RTK RTCM3. Delay: %f" % (delay))
                    self.correction_msg.header.stamp = self.get_clock().now()
                    self.correction_msg.data = out
                    print(self.correction_msg)
                    self.correction_pub.publish(self.correction_msg)
        except:
            print("ERROR: close serial")


if __name__ == '__main__':
    rclpy.init()
    emlid_correction_translator = EmlidCorrectionTranslator()
    rclpy.spin(emlid_correction_translator)
    emlid_correction_translator.destroy_node()
    rclpy.shutdown()