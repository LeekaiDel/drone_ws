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
        self.radius_of_landing_place = 0.2  # радиус метки посадки
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
        topic_cam = '/iris_rplidar/usb_cam/image_raw'
        drone_pose_topic = '/mavros/local_position/pose'

        # Global variables #
        self.drone_pose = PoseStamped()
        self.cv_img_raw = None
        self.bridge = CvBridge()
        self.landing_place_example_green_mask = None
        self.landing_place_example_blue_mask = None
        self.detected_green = None
        self.detected_blue = None
        self.ret = False
        self.drone_orientation_q = ()

        # Initing ros sub and pub #
        rospy.Subscriber(topic_cam, Image, self.ros_img_cb)
        rospy.Subscriber(drone_pose_topic, PoseStamped, self.drone_pose_cb)

        # Essentials initing #
        # Считываем и бинаризуем пример метки посадки из файла
        rospack = rospkg.RosPack()
        img_path = rospack.get_path('opencv_projects') + '/images/'
        self.ex_img_landing_place = cv2.imread(os.path.abspath(img_path + name_img_file_landing_place))  
        self.landing_place_ex_b_mask = cv2.inRange(self.ex_img_landing_place, self.LANDING_PLACE_IMG_B_TH_MIN, self.LANDING_PLACE_IMG_B_TH_MAX)
        self.landing_place_ex_b_mask = cv2.resize(self.landing_place_ex_b_mask, self.max_resize)
        self.landing_place_ex_g_mask = cv2.inRange(self.ex_img_landing_place, self.LANDING_PLACE_IMG_G_TH_MIN, self.LANDING_PLACE_IMG_G_TH_MAX)
        self.landing_place_ex_g_mask = cv2.resize(self.landing_place_ex_g_mask, self.max_resize)

        # Main cycle #
        while not rospy.is_shutdown():
            if self.ret and self.cv_img_raw is not None:
                # Получаем объект контура по указанному интервалу цвета
                point_land_blue = self.contour_finder(self.cv_img_raw, self.LANDING_PLACE_B_TH_MIN, self.LANDING_PLACE_B_TH_MAX)
                # print(point_land_blue.cords)
                # cv2.imshow("point_blue", point_land_blue.mask)

                # Получаем бъект контура по указанному интервалу цвета
                point_land_green = self.contour_finder(self.cv_img_raw, self.LANDING_PLACE_G_TH_MIN, self.LANDING_PLACE_G_TH_MAX)
                # print(point_land_green.cords)
                # cv2.imshow("point_green", point_land_green.mask)
                
                cut_contour_blue = self.cut_contour(point_land_blue, point_land_blue.cords)
                cut_contour_green = self.cut_contour(point_land_green, point_land_blue.cords)

                # cv2.imshow("cut_blue", cut_contour_blue)
                # cv2.imshow("cut_green", cut_contour_green)

                # Cравниваем маски с камеры и маску сделанную из файлов
                similarity_marker_blue = self.detect_marker(cut_contour_blue, self.landing_place_ex_b_mask)
                similarity_marker_green = self.detect_marker(cut_contour_green, self.landing_place_ex_g_mask)
                # print("similarity_marker_blue: " + str(similarity_marker_blue))
                # print("similarity_marker_green: " + str(similarity_marker_green))

                if similarity_marker_blue[0] - similarity_marker_blue[1] > 2900 and similarity_marker_green[0] - similarity_marker_green[1] > 2900:
                    # Получаем координаты маркера посадки в глобальной системе координат
                    correction_cords = self.calculate_pose_correction(point_land_green)
                    print("Correction_cords: " + str(correction_cords))
                else:
                    pass

            if cv2.waitKey(1) == 27:  # проверяем была ли нажата кнопка esc
                break
        

    # Функция выделения контуров
    def contour_finder(self, frame, val_min_hsv, val_max_hsv):
        # Cоздаём объект хранящий в себе основные параметры детектируемого объекта
        detect_obj = ContourObj()
        # Переводим картинку с камеры из формата BGR в HSV
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Делаем размытие картинки HSV
        hsv_img = cv2.blur(hsv_img, (4, 4))
        # Делаем бинаризацию картинки и пихаем её в переменную mask
        detect_obj.mask = cv2.inRange(hsv_img, val_min_hsv, val_max_hsv)
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


    # Функция вырезает детектируемый контур из кадра и возвращает его в бинаризованном виде с фиксированным размером кадра
    def cut_contour(self, contour_obj: ContourObj, cords):
        cut_contour_from_frame = contour_obj.mask[cords[1]: (cords[1] + cords[3]) + 1, cords[0]: (cords[0] + cords[2]) + 1]
        # Делаем фиксированный размер картинки в размерах заданных в переменной self.max_size
        cut_contour_from_frame = cv2.resize(cut_contour_from_frame, self.max_resize)
        return cut_contour_from_frame


    # Функция определяющая какой маркер обнаружен
    def detect_marker(self, cut_frame, origin_frame_bin):
        difference_val = 0
        similarity_val = 0
        try:
            for i in range(64):
                for j in range(64):
                    if cut_frame[i][j] == origin_frame_bin[i][j]:
                        similarity_val += 1
                    elif cut_frame[i][j] != origin_frame_bin[i][j]:
                        difference_val += 1
        except:
            similarity_val = 0
            difference_val = 0

        return (similarity_val, difference_val)


    # Функция коррекции положения дрона относительно сдетектированного маркера/объекта 
    def calculate_pose_correction(self, marker_obj):
        # Вычисляем локальные координаты метки в кадре камеры(измерение в пиксельных единицах)
        pix_x = (marker_obj.cords[0] + (marker_obj.cords[2] / 2)) - len(self.cv_img_raw[0]) / 2
        pix_y = -((marker_obj.cords[1] + (marker_obj.cords[3] / 2)) - len(self.cv_img_raw) / 2)
        pix_on_meter = self.recalculation_cords(marker_obj.mask, self.radius_of_landing_place)
        # print("pix_x: " + str(pix_x), "pix_y: " + str(pix_y))
        local_correction_cords = np.array([float(pix_x) / pix_on_meter, float(pix_y) / pix_on_meter, 0.0])
        # print("local_transform_cords: " + str(local_correction_cords)) 
        # Считаем углы поворота дрона из кватерниона в углы эйлера
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(self.drone_orientation_q)
        # print("yaw: " + str(yaw))
        glob_X, glob_Y = self.transform_cord(yaw, local_correction_cords)  # пересчитываем найденные локальные координаты в глобальные
        # print("X = %s, Y = %s" % (glob_X, glob_Y))
        return (glob_X, glob_Y)


    # Функция вычисления количества пикселей на метр
    def recalculation_cords(self, all_binary, radius_of_landing_place):
        list_interval = []
        for i in np.arange(0, all_binary.shape[1]):
            list_interval.append(np.sum(all_binary[:, i] == 255))
            # print i, AllBinary[:, i]
        # Находим коэффициент пиксель на метр
        # print max(LIST)
        pixel_on_meter = float(max(list_interval)) / radius_of_landing_place
        # print("pixel_on_meter: " + str(pixel_on_meter))
        return pixel_on_meter


    # Функция преобразования локальных координат в глобальные координаты
    def transform_cord(self, W, cords):
        X = (math.cos(W) * (self.drone_pose.pose.position.x * math.cos(W) + self.drone_pose.pose.position.y * math.sin(W))) + (math.sin(W) * (self.drone_pose.pose.position.x * math.sin(W) - self.drone_pose.pose.position.y * math.cos(W))) + (cords[0] * math.cos(W) - cords[1] * math.sin(W))
        Y = (math.sin(W) * (self.drone_pose.pose.position.x * math.cos(W) + self.drone_pose.pose.position.y * math.sin(W))) - (math.cos(W) * (self.drone_pose.pose.position.x * math.sin(W) - self.drone_pose.pose.position.y * math.cos(W))) + (cords[0] * math.sin(W) + cords[1] * math.cos(W))
        return X, Y


    # Функция получает на вход картинку из топика камеры и переводит ее в cv формат
    def ros_img_cb(self, data: Image):
        self.cv_img_raw = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.ret = True


    # Функция считывания текущего положения дрона
    def drone_pose_cb(self, data: PoseStamped):
        self.drone_pose = data
        self.drone_orientation_q = (
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w)


if __name__ == "__main__":
    rospy.init_node('detector_landing_place')
    DetectorLandingPlace()