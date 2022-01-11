#!/usr/bin/env python3
#coding=utf8

# import cv2
import numpy as np
import rospy
import tf2_ros
import matplotlib.pyplot as plt

from nav_msgs.msg import OccupancyGrid

# функция перевода координат рос в координаты ячеек в массиве
def cords2cell(cords, resolution):
    """
    :param cords:
    :param resolution:
    :return:
    """
    if resolution <= 0.000001:
        return [0, 0]
    return [int(cords[0] / resolution),
            int(cords[1] / resolution)]

# функция нахождения эвристической длины
def valid_path(pose_origin, goal_):
    pass

def ros_to_occupancy_grid(position, width, height, resolution, origin):
    """
    ROS position -> OccupancyGrid cell pixel
    :param position:
    :param width:
    :param height:
    :param resolution:
    :param origin:
    :return:
    """
    pixel_x = height - float(position[1] - origin[1]) / resolution
    pixel_y = width - float(position[0] - origin[0]) / resolution
    return [int(pixel_x), int(pixel_y)]


class cv_map_parser:
    def __init__(self):
        topic_map = "/map"
        self.gridMap = OccupancyGrid()
        self.last_time = rospy.get_time()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.tfCords = []   # хранит X и Y из tf преобразований целевого фрейма 'base_link' относительно фрейма 'odom'
        self.cartographer_map = None
        # init subscribers
        rospy.Subscriber(topic_map, OccupancyGrid, self.map_cb)


    def map_cb(self, msg):
        self.gridMap = msg
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.length = len(msg.data)
        self.origin_position = [msg.info.origin.position.x, msg.info.origin.position.y, msg.info.origin.orientation]

        # print(width, height, resolution)
        # print(self.origin_position)

        # create empty image
        self.cartographer_map = np.zeros((self.width, self.height), np.uint8) * 255
        ox, oy = [], []
        # convert from OccupancyGrid
        for i in range(1, self.height):
            for j in range(1, self.width):
                value = msg.data[(i - 1) * self.width + j]
                if value < 50:                                   # 55
                    self.cartographer_map[self.width - j, self.height - i] = 0     # 255
                else:
                    ox.append(self.width - j)
                    oy.append(self.height - i)   
                    self.cartographer_map[self.width - j, self.height - i] = 255   # 0
        print(np.shape(self.cartographer_map))
        fig, ax = plt.subplots()
        ax.imshow(self.cartographer_map)
        ax.grid(True)
        plt.show()
        # print("yes")
        ##  filters for map   ##
        # self.kernel = np.ones((1, 1), 'uint8')
        # self.cartographer_map = cv2.erode(self.cartographer_map, self.kernel, iterations=1)

        # try:
        #     self.tfTranslation = self.tfBuffer.lookup_transform('base_link', 'odom', rospy.Time())
        #     self.tfCords = [self.tfTranslation.transform.translation.x, self.tfTranslation.transform.translation.y]
        #     print(self.tfCords)
        #     a = cords2cell(self.tfCords, self.resolution)
        #     print(a)
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     pass

        # if self.cartographer_map is not None:
        #     rgb_map_debug = np.dstack((self.cartographer_map, self.cartographer_map, self.cartographer_map))
        #     cv2.imshow("map", rgb_map_debug) #cv2.resize(rgb_map_debug, (self.height, self.width))

        #     if cv2.waitKey(1) == 27:  # проверяем была ли нажата кнопка esc
        #         cv2.destroyAllWindows()

        # print("Ping => %s" %(rospy.get_time() - self.last_time))
        # self.last_time = rospy.get_time()

if __name__ == '__main__':
    rospy.init_node("cv_map_parser")    
    cv_map_parser()
    plt.close()    
    rospy.spin()
