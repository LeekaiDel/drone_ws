#!/usr/bin/env python3
import rospy
import rospkg
import cv2
import numpy as np
import math
import tf
import os

from cv_bridge import CvBridge
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image


# Класс хранящий основные параметры найденных контуров
class ContourObj():
    # Конструктор
    def __init__(self):
        self.name = None
        self.cords = []
        self.mask = []


class DetectorLandingPlace():
    def __init__(self):
        # Static variables #
        self.radius_of_point_land = 0.2  # радиус метки посадки
        name_img_file_landing_place = 'landing_place.png'
        self.max_resize = (64, 64)
        # Диапазон синего круга в метке посадки загруженной из папки с проектом
        self.LANDING_PLACE_IMG_B_TH_MIN = (0, 0, 0)
        self.LANDING_PLACE_IMG_B_TH_MAX = (255, 0, 0)
        # Диапазон зеленого круга в метке посадки загруженной из папки с проектом
        self.LANDING_PLACE_IMG_G_TH_MIN = (0, 0, 0)
        self.LANDING_PLACE_IMG_G_TH_MAX = (0, 255, 255)
        # Диапазон синего круга в метке посадки детектируемой камерой
        self.LANDING_PLACE_B_TH_MIN = (61, 167, 0)
        self.LANDING_PLACE_B_TH_MAX = (255, 255, 255)
        # Диапазон зеленого круга в метке посадки детектируемой камерой
        self.LANDING_PLACE_G_TH_MIN = (43, 86, 197)
        self.LANDING_PLACE_G_TH_MAX = (66, 255, 243)

        # Topic names #
        topic_cam = ''
        drone_pose_topic = '/mavros/local_position/pose'

        # Global variables #
        self.drone_pose = PoseStamped()
        self.cv_img_raw = None
        self.bridge = CvBridge()
        self.landing_place_example_green_mask = None
        self.landing_place_example_blue_mask = None
        self.detected_green = None
        self.detected_blue = None
        # Initing ros sub and pub #
        rospy.Subscriber(topic_cam, Image, self.frame_down_cb)
        rospy.Subscriber(drone_pose_topic, PoseStamped, self.drone_pose_cb)

        # Essentials initing #
        # Считываем и бинаризуем пример метки посадки из файла
        rospack = rospkg.RosPack()
        img_path = rospack.get_path('opencv_drone') + '/images/'
        self.ex_img_landing_place = cv2.imread(os.path.abspath(img_path + name_img_file_landing_place))  
        self.landing_place_ex_b_mask = cv2.inRange(self.ex_img_landing_place, self.LANDING_PLACE_IMG_B_TH_MIN, self.LANDING_PLACE_IMG_B_TH_MAX)
        self.landing_place_ex_b_mask = cv2.resize(self.landing_place_ex_b_mask, self.max_resize)
        self.landing_place_ex_g_mask = cv2.inRange(self.ex_img_landing_place, self.LANDING_PLACE_IMG_G_TH_MIN, self.LANDING_PLACE_IMG_G_TH_MAX)
        self.landing_place_ex_g_mask = cv2.resize(self.landing_place_ex_g_mask, self.max_resize)

        # Main cycle #
        while not rospy.is_shutdown():
            # Получаем объект контура по указанному интервалу цвета
            point_land_blue = self.contour_finder(self.cv_img_raw, self.LANDING_PLACE_B_TH_MIN, self.LANDING_PLACE_B_TH_MAX)
            # print(point_land_blue.cords)
            cv2.imshow("point_blue", point_land_blue.mask)

            # Получаем бъект контура по указанному интервалу цвета
            point_land_green = self.contour_finder(self.cv_img_raw, self.LANDING_PLACE_G_TH_MIN, self.LANDING_PLACE_G_TH_MAX)
            # print(point_land_green.cords)
            cv2.imshow("point_green", point_land_green.mask)
            
            if cv2.waitKey(1) == 27:  # проверяем была ли нажата кнопка esc
                break
        
    # Функция выделения контуров
    def contour_finder(self, frame, var_min_bgr, var_max_bgr):
        # Cоздаём объект хранящий в себе основные параметры детектируемого объекта
        detect_obj = ContourObj()

        # Переводим картинку с камеры из формата BGR в HSV
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Делаем размытие картинки HSV
        hsv_img = cv2.blur(hsv_img, (4, 4))

        # Делаем бинаризацию картинки и пихаем её в переменную mask
        detect_obj.mask = cv2.inRange(hsv_img, var_min_bgr, var_max_bgr)
        # cv2.imshow('mask', mask)

        # Уменьшаем контуры белых объектов - делаем две итерации
        detect_obj.mask = cv2.erode(detect_obj.mask, None, iterations = 3)
        # cv2.imshow("Erode", mask)

        # Увеличиваем контуры белых объектов (Делаем противоположность функции erode) - делаем две итерации
        detect_obj.mask = cv2.dilate(detect_obj.mask, None, iterations = 3)

        # Ищем контуры в результирующем кадре
        contours, hierarchy = cv2.findContours(detect_obj.mask, cv2.RETR_TREE , cv2.CHAIN_APPROX_NONE)

        # Проверяем найдены ли контуры в кадре
        if len(contours):
            # Сортируем элементы массива контуров по площади по убыванию
            contours = sorted(contours, key = cv2.contourArea, reverse = True)

            # Выводим все контуры на изображении
            # cv2.drawContours(frame, contours, -1, (0, 180, 255), 1)  # cv.drawContours(кадр, массив с контурами, индекс контура, цветовой диапазон контура, толщина контура)

            # Получаем координаты прямоугольника описанного относительно контура
            detect_obj.cords = cv2.boundingRect(contours[0])  # возвращает кортеж в формате  (x, y, w, h)
            return detect_obj

        else:
            return detect_obj


    # Функция получает на вход картинку из топика камеры и переводит ее в cv формат
    def ros_img_cb(self, data: Image):
        ros_img = data
        self.cv_img_down = self.bridge.imgmsg_to_cv2(ros_img, "bgr8")


    # Функция считывания текущего положения дрона
    def drone_pose_cb(self, data: PoseStamped):
        self.drone_pose = data


if __name__ == "__main__":
    rospy.init_node('detector_landing_place')
    DetectorLandingPlace()