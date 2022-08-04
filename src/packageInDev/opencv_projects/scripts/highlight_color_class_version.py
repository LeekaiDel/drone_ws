#!/usr/bin/env python3

import cv2
from sympy import Q 
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class HighlightColor():
    def __init__(self):
        # Topics name
        self.cam_topic = "/r200/image_raw"
        # Global variables
        self.cv_img_raw = None
        self.bridge = CvBridge()
        self.ret = False
        # Создаем подписку на топик
        rospy.Subscriber(self.cam_topic, Image, self.img_cb, queue_size=10)
        # Создаем панель фейдеров для подбора значений выделения цвета
        # Создам пустое окно с именем result
        cv2.namedWindow('result')
        # Создаём в окне result бегунки для задания порогов цвета
        cv2.createTrackbar('minb', 'result', 0, 255, self.nothing)
        cv2.createTrackbar('ming', 'result', 0, 255, self.nothing)
        cv2.createTrackbar('minr', 'result', 0, 255, self.nothing)

        cv2.createTrackbar('maxb', 'result', 0, 255, self.nothing)
        cv2.createTrackbar('maxg', 'result', 0, 255, self.nothing)
        cv2.createTrackbar('maxr', 'result', 0, 255, self.nothing)
        
        # Главный цикл
        while not rospy.is_shutdown():
            # Проверяем приходила ли картинка через флажок ret
            if self.ret:
                hsv_img = cv2.cvtColor(self.cv_img_raw, cv2.COLOR_BGR2HSV)
                # cv2.imshow('frame', hsv_img)
                # Получаем данные положения фейдеров для каждого канала цвета
                minb = cv2.getTrackbarPos('minb', 'result')
                ming = cv2.getTrackbarPos('ming', 'result')
                minr = cv2.getTrackbarPos('minr', 'result')

                maxb = cv2.getTrackbarPos('maxb', 'result')
                maxg = cv2.getTrackbarPos('maxg', 'result')
                maxr = cv2.getTrackbarPos('maxr', 'result')

                # Делаем размытие картинки в формате HSV
                blur_hsv_img = cv2.blur(hsv_img, (4, 4))
                # cv2.imshow('frame_blur', blur_hsv_img)

                # Делаем бинаризацию картинки и пихаем её в переменную mask
                mask = cv2.inRange(blur_hsv_img, (minb, ming, minr), (maxb, maxg, maxr))
                # Уменьшаем контуры белых объектов - делаем две итерации
                maskEr = cv2.erode(mask, None, iterations=2)
                # cv2.imshow("Erode", maskEr)
                # Увеличиваем контуры белых объектов (Делаем противоположность функции erode) - делаем две итерации
                maskDi = cv2.dilate(maskEr, None, iterations=2)
                # cv2.imshow('Dilate', maskDi)
                # Накладываем полученную маску на картинку с камеры переведённую в формат HSV
                result = cv2.bitwise_and(self.cv_img_raw, self.cv_img_raw, mask = mask)
                # result = cv.resize(result, (500, 500))
                cv2.imshow('result', result)


                if cv2.waitKey(1) == 27: 
                    break

    # Функция принимает на вход из топика картинку из камеры
    def img_cb(self, msg: Image):
        self.cv_img_raw = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.ret = True
        # cv2.imshow('frame_raw', self.cv_img_raw)


    # Эта функция ничего не делает (Логично блять)
    def nothing(self, x):
        pass


if __name__ == "__main__":
    rospy.init_node("highlight_color_node")
    HighlightColor()
