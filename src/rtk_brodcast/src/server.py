#!/usr/bin/env python
# coding: utf-8

"""
Программа для трансляции поправки с BASE RTK через шириковещательный канал.
Данные принимаются через 2 канала на выбор:
1. TCP/IP client
2. UART

Транслировать: UDP

PORT по умолчанию 9000
"""

from socket import *
import time
from threading import Thread
import serial
import os


use_tcp = False

UDP_MASK = '192.168.128.255'
UDP_PORT = 9000

TCP_PORT = 9002
TCP_IP = "192.168.128.9"

PORT = "/dev/ttyACM0"
baud = 115200

def send_brodcast_msgs(data):
    """
    Отправляем данные о дроне всем клиетам сети
    :param data: текущее состояние дрона
    :type data: DroneInfo
    """
    global udp_clien_sock
    udp_clien_sock.sendto(data, (UDP_MASK, UDP_PORT))

### Server
def receive_from_server():
    """
    receive hit

    :return:
    """
    global server, message
    while True:
        client_socket, address = server.accept()
        print 'Accepted connection from {}:{}'.format(address[0], address[1])
        request = client_socket.recv(1024)
        client_socket.close()

        message = request

if __name__ == '__main__':

    # init client params
    udp_clien_sock = socket(AF_INET, SOCK_DGRAM)  # Internet,UDP
    udp_clien_sock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
    udp_clien_sock.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)


    if use_tcp:
        # create an ipv4 (AF_INET) socket object using the tcp protocol (SOCK_STREAM)
        print("Connect to %s:%s" % (TCP_IP, TCP_PORT))
        client = socket(AF_INET, SOCK_STREAM)
        # connect to the TCP client
        try:
            client.connect((TCP_IP, TCP_PORT))
        except:
            print "server not found"
            exit()
    else:
        try:
        	ser = serial.Serial(
            		port=PORT,
            		baudrate=baud,
            		parity=serial.PARITY_ODD,
            		stopbits=serial.STOPBITS_TWO,
            		bytesize=serial.SEVENBITS)
        	ser.isOpen()
    	except:
        	print("port not found")
		exit()

    prev_time = time.time()

    try:
        while True:

            delay = time.time() - prev_time
            prev_time = time.time()

            if use_tcp:
                # get data from TCP
                response = client.recv(4096)
                print("TCP: Get RTK RTCM3. Delay: %f" %(delay))
                send_brodcast_msgs(response)
            else:
                # get data from serial
                out = ''
                while ser.inWaiting() > 0:
                    out += ser.read(1)
                if out != '':
                    print("Serial: Get RTK RTCM3. Delay: %f" % (delay))
                    send_brodcast_msgs(out)
    except:
        if use_tcp:
            client.close()
            print("ERROR: close client")
        else:
            print("ERROR: close serial")
