#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from laser_geometry import LaserProjection
import os


class Scan_Parser():
    def __init__(self):
        self.laserConverter = LaserProjection()
        self.pc2ScanPub = rospy.Publisher(
            "/scan_into_pc", PointCloud2, queue_size=1)
        self.laserSub = rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        self.points_array = None

    def scan_cb(self, msg):
        print("recv")
        cloud_out = self.laserConverter.projectLaser(msg)
        self.pc2ScanPub.publish(cloud_out)
        self.points_array = pc2.read_points_list(cloud_out)
        os.system("clear")
        for point in self.points_array:
            print(point)


if __name__ == "__main__":
    rospy.init_node('scan_parser', anonymous=True)
    scan_parser = Scan_Parser()

    while not rospy.is_shutdown():
        pass
