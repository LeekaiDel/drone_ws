#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from laser_geometry import LaserProjection
import os
from visualization_msgs.msg import Marker
import math
from copy import deepcopy
import matplotlib.pyplot as plt
import numpy as np
import pcl
import sys
import struct
# import pcl_msgs
import ctypes


class Scan_Parser():
    def __init__(self):
        self.laserConverter = LaserProjection()
        self.pc2ScanPub = rospy.Publisher(
            "/scan_into_pc", PointCloud2, queue_size=1)
        self.marker_pub = rospy.Publisher("/room_viz", Marker, queue_size=1)
        self.laserSub = rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        self.points_array = None
        self.d_functions = None
        self.timer = rospy.Timer(rospy.Duration(0.1), self.py_plot_cb)

    def py_plot_cb(self, timer=None):
        if self.d_functions is not None:
            # Рисуем графики
            ax1 = plt.subplot2grid((1, 1), (0, 0))
            ax1.plot(self.d_functions)
            ax1.set_ylabel('x, m')
            ax1.set_xlabel('angle, grad')
            ax1.legend()
            ax1.grid()
            plt.pause(1.0 / 20.0)

    def scan_cb(self, msg):
        cloud_out = self.laserConverter.projectLaser(msg)
        # self.pc2ScanPub.publish(cloud_out)
        self.points_array = pc2.read_points_list(cloud_out)
        PointCloudBlyat = pcl.PointCloud()
        pc_list = []
        for p in self.points_array:
            pc_list.append([p[0], p[1], p[2]])
        PointCloudBlyat.from_list(pc_list)
        sor = PointCloudBlyat.make_voxel_grid_filter()
        sor.set_leaf_size(0.01, 0.01, 0.01)
        cloud_filtered = sor.filter()
        # pcl_ros = pc2.create_cloud_xyz32(cloud_out.header, cloud_filtered)
        print("pcl_ros: " + str(type(cloud_filtered)))
        # self.pc2ScanPub.publish(pcl_ros)

        # self.marker_pub.publish(self.make_marker(self.points_array, 1))
        # os.system("clear")
        # self.d_functions = self.radius_filter(self.points_array, 5.0)
        # self.marker_pub.publish(self.make_contour(
        #     self.d_functions, 2, (0.0, 1.0, 0.0)))
        # print("\tDebug: " + str(self.d_functions))

    def radius_filter(self, point_list: list, radius: float) -> list:
        array = list()
        r_array = list()
        for point in point_list:
            r = math.sqrt(point.x**2 + point.y**2 + point.z**2)
            r_array.append(r)

        for i in range(len(r_array)):
            if r_array[i] < radius:
                array.append(point_list[i])
        return r_array

    def make_contour(self, point_list, id, rgb):
        marker = Marker()
        marker.header.frame_id = "laser"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.id = id
        marker.scale.x = 0.1
        # marker.scale.y = 0.2
        # marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.points.extend(point_list)
        marker.points.append(point_list[0])

        return marker


if __name__ == "__main__":
    rospy.init_node('scan_parser', anonymous=True)
    scan_parser = Scan_Parser()

    while not rospy.is_shutdown():
        pass
