#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import copy

import math
import matplotlib.pyplot as plt
import tf

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Point, PoseStamped
from visualization_msgs.msg import *
from geometry_msgs.msg import TwistStamped, Twist
from drone_msgs.msg import *
from std_msgs.msg import Bool

# Параметры планировщика
use_gps_flag = True
range_of_sectors = 12           # Количество секторов лидара
max_dist_lidar = 8.0           # Максимальная длина лидара

invert_sonar = False

state_planner = True            # вкл/выкл плаанировщика
state_interpolete = False
r_max = 1.5                    # максимально допустимое расстояния до препятствия
r2_max = 1.5                   # Радиус выхода из неуустойчивого режима
k_dist = 0.3                   # коэффициент для поиска оптимального направления
dist_fly = 1.0			# дистанция облёта
r_p = 0                         # вектор навравления куда летим
max_height = 3.0                # максимальная высота

# Инициализация глобальных переменных
sonar_data = list()
sectors = list()                # Массив секторов

initFlag = False
current_pos = [0.0, 0.0, 0.0]   # Массив с текущей позицией дрона
current_course = 0.0;           # текущий курс дрона
target_pos = Goal()             # Целевая точка дрона
velocity = [0.0, 0.0, 0.0]      # Скорость дрона
interpolite_point = [None, None, None] # Текущая интерполированная точка
angle_goal_point = 0.0       # Угол между курсом и целевой точкой
angle_between_vel = 0.0
angle_between_course_and_goal = 0.0

# Список топиков
if use_gps_flag:
  topic_velocity = "/mavros/local_position/velocity"
else:
  topic_velocity = "/drone/velocity"

sonar_topic = "/scan"
goal_sub_topic = "/goal_pose"
goal_pub_topic = "/goal_pose_to_reg"
pose_topic = "/mavros/local_position/pose"

pub_goal = rospy.Publisher("/goal_pose_to_reg", Goal, queue_size=10)
pub_marker = rospy.Publisher("/marker_target_point", Marker,queue_size=10)


# ---------------------------------------------------------------------------------------------------------------------
# ---------- Геометрические функции

def norm_angle(data):
    res = data

    if res > math.pi:
        res -= 2 * math.pi
    if res < -math.pi:
        res += 2 * math.pi

    return res


def rotate_vect(a, b, rot, dist):
    """
    Поворачиваем b относительно a на угол rot
    :type a: list
    :type b: Goal
    :type rot: float
    :type dist: float
    :param a:
    :param b:
    :param rot:
    :return: возвращаем точку повёрнутую на нужный угол

    """
    rotate = np.array([[math.cos(-rot), -math.sin(-rot)],
                       [math.sin(-rot), math.cos(-rot)]])

    pos = np.array([[b.pose.point.x - a[0]],
                    [b.pose.point.y - a[1]]])
    try:
        val = np.dot(rotate, pos)
        normVec = val / np.linalg.norm(val) * dist
        normVec[0] += a[0]
        normVec[1] += a[1]
    except:
        normVec = [b.pose.point.x, b.pose.point.y]

    b.pose.point.x = normVec[0]
    b.pose.point.y = normVec[1]

    return b


def get_distance_to(a, b):
    """
    Возврааем дистанцию до целевой точки
    :type a: list
    :type b: Goal
    :type dist: float
    :param a: текущая позиция
    :param b: целевая точка
    :param dist: вистанция между точками
    :return: dist
    """

    pos = np.array([[b.pose.point.x - a[0]],
                    [b.pose.point.y - a[1]]])
    dist = np.linalg.norm(pos)
    return dist


def angle_between(vec1, vec2):
    """
      Функция возвращает угол между двумя двухмерными векторами.

    :param vec1:
    :param vec2:
    :return:
    """

    x = (vec2[1] - vec1[1])  # y
    y = -(vec2[0] - vec1[0])  # -x
    res = math.atan2(x, y) + math.pi

    if res > math.pi:
        res -= 2 * math.pi
    if res < -math.pi:
        res += 2 * math.pi
    return res

def lerp_point(current_pos, newPoint, step):
    """
    Интерполируем точку куда лететь
    :type target_pos: Goal
    :type current_pos: []
    :type step: float
    :param current_pos: текущая позиция
    :param target_pos: желаемая позиция
    :param step: растояние на которое нужно интерполировать
    :return:
    """
    global interpolite_point
    # вектор куда лететь
    vec = [newPoint.pose.point.x - current_pos[0],
           newPoint.pose.point.y - current_pos[1],
           newPoint.pose.point.z - current_pos[2]]

    dist = np.linalg.norm(vec)

    # если добрались до конечной, то отключаем интерполяцию
    if dist < step:
        return newPoint
 #   else:
        # инача интерполируем следующую точку
        # Умножаем нормализованный вектор на скаляр
        val = vec / dist * step

        newPoint.pose.point.x = val[0] + current_pos[0]
        newPoint.pose.point.y = val[1] + current_pos[1]
    # newPoint.pose.point.z = val[2] + current_pos[2]

        interpolite_point = [val[0] + current_pos[0],
                             val[1] + current_pos[1],
                             val[2] + current_pos[2]]

    return newPoint

# ---------------------------------------------------------------------------------------------------------------------
# ---------- Функции подписки на топики

def callbackSonar(data):
    """
    Чатаем данные с сонара и определяем свободную зону на карте
    :type data: LaserScan

    :return:
    """
    global sonar_data
    #get data sectors
    sonar_data = data.ranges

def callbackCurrentPose(data):
    """
    :type data: PoseStamped
    :param data:
    :return:
    """
    global current_pos, current_course, angle_goal_point, target_pos

    current_pos = [data.pose.position.x, data.pose.position.y, data.pose.position.z]

    # Получаем текущий курс дрона
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion((data.pose.orientation.x,
                                                                   data.pose.orientation.y,
                                                                   data.pose.orientation.z,
                                                                   data.pose.orientation.w))
    current_course = yaw


def callbackTargetPose(data):
    """
    Подписываемся на сообщение от глобального планировщика.
    Заносим данные в глобальную переменную
    @:type ta

    @type data: Goal

    :param data:
    :return:
    """
    global target_pos, \
        initFlag, \
        interpolite_point, \
        current_pos, \
        pub_goal, \
        pub_marker, \
        state_planner, \
        max_height

    # climp the height
    data.pose.point.z = data.pose.point.z if data.pose.point.z < max_height else max_height
    if not state_planner:
        pub_goal.publish(data)
        pub_marker.publish(setup_marker(data.pose.point, True))
        return
    prev_vec = [target_pos.pose.point.x - current_pos[0],
                target_pos.pose.point.y - current_pos[1]]
    prev_point_dist = np.linalg.norm(prev_vec)

    new_vec = [data.pose.point.x - current_pos[0],
               data.pose.point.y - current_pos[1]]
    new_dist = np.linalg.norm(new_vec)

    if new_dist < prev_point_dist:
        # print("Сбрасываем путь")
        interpolite_point = [None, None, None]

    target_pos = data
    initFlag = True

def callbackVelocity(data):
    """
    :param data:
    :type data: TwistStamped
    :return:
    """
    global velocity
    velocity[0] = data.twist.linear.x
    velocity[1] = -data.twist.linear.y
    velocity[2] = data.twist.linear.z

def callbackStatePlanner(data):
    """
    :type data: LocalPlannerState
    :param data:
    :return:
    """
    global state_planner, r_max, k_dist
    state_planner = data.state
    r_max = data.max_dist
    k_dist = data.k

    # print ("state: %s \nr_max: %s \nk: %s \n" %(state_planner,r_max,k_dist))

def sonar_to_sectors(sonar_data):
    """"
    Делим сырые  данные на сектора
    Центральный сектор направлен вдоль целевой точки

    :return массив секторов с препятствиями
    """
    global current_pos, target_pos, current_course, angle_goal_point, angle_between_course_and_goal

    # поворачиваем данные относительно куда целевой точки
    invert = 1.0
    if invert_sonar:
        invert = -1.0
    try:
        srez = int(len(sonar_data) / (2*math.pi) * angle_between_course_and_goal * invert)
        sonar_data = sonar_data[-srez:] + sonar_data[:-srez]
    except ValueError:
        pass

    #избавляемся от inf
    without_inf = list()
    for k in range(0,len(sonar_data)):
        if np.isinf(sonar_data[k]):
            without_inf.append(max_dist_lidar)
        else:
            without_inf.append(sonar_data[k])
    # инвертируем сонар без inf
    sonar_data = without_inf

    # количество значений в секторе
    samples_of_sector = len(sonar_data) / range_of_sectors

    sectors = list()
    for i in range(0, range_of_sectors):
        if i == 0:
            data_of_sector = sonar_data[-(samples_of_sector / 2):]
            data_of_sector += sonar_data[:samples_of_sector * (i + 1) - (samples_of_sector / 2)]
        else:
            data_of_sector = np.asarray(sonar_data[(samples_of_sector * i)-(samples_of_sector/2):(samples_of_sector * (i + 1))-(samples_of_sector/2)])

        target_angle = -np.pi+((2*np.pi) / range_of_sectors *i)
        target_angle = -target_angle
        data_sector = np.min(data_of_sector)
        data_sector = max_dist_lidar if math.isinf(data_sector) else data_sector
        sectors.append([data_sector,np.degrees(target_angle)])

        # # drag sector data
        # size_secrot = ""
        # for i in range(int(data_sector)+1):
        #     size_secrot += "=="
        # print(str(np.degrees(target_angle)) + ": " + size_secrot)
        # print("max:" +  str(min(sectors)))
    return sectors

def setup_marker(pose, freeFlag):
    """
    Настройка маркера для отображения в rviz
    :type pase: Point
    :param pose:
    :return:
    """
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.header.stamp = rospy.get_rostime()
    marker.ns = "target_point"
    marker.id = 0
    marker.action = 0
    marker.pose.orientation.x = 0
    marker.pose.orientation.y = 0
    marker.pose.orientation.z = 0
    marker.pose.orientation.w = 1.0

    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    marker.type = Marker.SPHERE
    marker.color.r = 0.0 if freeFlag else 1.0
    marker.color.g = 1.0 if freeFlag else 0.0
    marker.color.b = 0.0
    marker.color.a = 0.8
    marker.pose.position.x = pose.pose.point.x
    marker.pose.position.y = pose.pose.point.y
    marker.pose.position.z = pose.pose.point.z

    return marker


# ---------------------------------------------------------------------------------------------------------------------
# ---------- Функции планировщика

flag_free = False
unstable_mode = False

def get_free_direct(sectors, target_dist, r_max):
    """
    Определяем направление  и расстояние куда лететь
    Основная логина планировщика здесь
    :param sectors:
    :param r_max:
    :return:
    """
    global k_dist, angle_between_vel, flag_free, angle_goal_point, unstable_mode

    vel_sector = int(angle_between_vel / (math.pi / (range_of_sectors / 2)))
    print("vel_sector", vel_sector)
    # выключаем планировщик если:
    # 1. Расстоние по курсу больше заданного
    # 2. Расстояние до целевой точки меньше чем до препятствия
   # and sectors[(len(sectors)/2) - vel_sector] > r_max)
    flag_free = True

    # if (sectors[(len(sectors)/2)-vel_sector] > r_max):
    if (sectors[len(sectors)/2] > r_max and sectors[(len(sectors)/2)-vel_sector] > r_max) \
            or (target_dist < (sectors[len(sectors)/2]-(r_max/2)) and target_dist <(sectors[(len(sectors) / 2) - vel_sector]-(r_max / 2))):
        print('planner off')
        # return 0.0, 0.0
        flag_free = True
    else:
        flag_free = False
        unstable_mode = True

    for i in range(len(sectors)):
        if sectors[i] < r_max*0.6:
            flag_free = False
            unstable_mode = True


    # # Не выходим из неустойчивого режима, пока не выйдем из радиуса r2_max
    # if unstable_mode == True:
    #     flag_free = False
    #
    #     safe_zone = True
    #     for i in range(len(sectors)):
    #         if sectors[i] < r2_max:
    #             safe_zone = False
    #
    #     if safe_zone == True:
    #         unstable_mode = False
    #         flag_free = True

    if flag_free:
        print("return")
        return 0.0, 0.0

    direction = 0.0
    index = len(sectors)/2

    # определяем направление куда повернуть точку
    # 1. создаём 2 вектора слева и справа от целевой точки
    left = sectors[index-1:0-1:-1]
    right = sectors[index+1:]

    # 2. Смотрим с какой сторроны больше всего расстояние, туда и летим
    summLeft = np.sum(left)
    summRight = np.sum(right)
    if  summLeft > summRight:
        direction = -1.0
    else:
        direction = 1.0

    angle = 0.0
    dist = 0.0

    # 3. определяем близжайший угол с большим расстоянием
    if direction < 0:
        # находим близжайшее направление где расстония больше минимального * k_dist
        for i in range(0,len(left)):
            if left[i] > r_max * k_dist:
                angle = np.pi * 2 / len(sectors) * i+1
                dist = sectors[len(sectors) / 2] * k_dist
                # dist = left[i] * k_dist
                return [-angle * direction, dist]

    elif direction > 0:
        # находим близжайшее направление где расстония больше минимального * k_dist
        for i in range(0,len(right)):
            if right[i] > r_max * k_dist:
                angle = np.pi * 2 / len(sectors) * i+1
                dist = sectors[len(sectors)/2] * k_dist
                # dist = right[i] * k_dist
                return [angle * direction, dist]

    # 4. Если не нашли угол удовлетворяющий условию, просто летим в самый свободный
    maxVal = np.max(sectors)
    for i in range(0, len(sectors)):
        if sectors[i] == maxVal:
            angle = -np.pi * 2 / len(sectors) * i + 1
            dist = sectors[len(sectors) / 2] * k_dist
            # dist = maxVal * k_dist
            return [angle, dist]

    return [angle, dist]


def get_current_pos(listener):
    """
    Получаем текущие координаты дроона
    :param listener:
    :return:
    """
    global  current_pos, angle_goal_point, current_course, target_pos, velocity, angle_between_vel
    try:
        (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        current_pos[0] = trans[0]
        current_pos[1] = trans[1]
        current_pos[2] = trans[2]

        # Получаем текущий курс дрона
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
        current_course = yaw

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print("ERROR")

def pub_vel_angle():
    global velocity, current_pos
    engle = angle_between([0.0, 0.0, 0.0], velocity)
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    (x, y, z, w) = tf.transformations.quaternion_from_euler(0.0, 0.0, engle)
    pose.pose.position.x = current_pos[0]
    pose.pose.position.y = current_pos[1]
    pose.pose.position.z = current_pos[2]

    pose.pose.orientation.x = x
    pose.pose.orientation.y = y
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w
    return pose
# ---------------------------------------------------------------------------------------------------------------------
# ---------- Основная функция

def main():
    """
    Основной цикл узла ROS.

    :return: код завершения программы
    """
    global current_pos, current_course, angle_goal_point, interpolite_point, angle_between_course_and_goal
    global velocity, free_dir, sonar_data, r_max, sectors, target_pos, angle_between_vel, angle_goal_point

    rospy.init_node('unstable_planner_node', anonymous=True)

    rate = rospy.Rate(10) # 10 hz

    # Подписываемся на топики
    listener = tf.TransformListener()
    rospy.Subscriber(sonar_topic, LaserScan, callbackSonar)
    rospy.Subscriber(goal_sub_topic, Goal,callbackTargetPose)
    # Раскоментить, если нужна скорость
    rospy.Subscriber(topic_velocity, TwistStamped, callbackVelocity)
    # Управление планироващиком
    rospy.Subscriber("local_planner/state", LocalPlannerState, callbackStatePlanner)
    vel_ang_pub = rospy.Publisher("velocity/angel", PoseStamped, queue_size=1)
    state_pub = rospy.Publisher("planner_state", Bool, queue_size=1)

    if use_gps_flag:
        rospy.Subscriber(pose_topic, PoseStamped, callbackCurrentPose)
    vel_list = list()

    # Основной цикл, работает, пока запущен рос
    while not rospy.is_shutdown():

        if not state_planner:
            continue

        if not use_gps_flag:
            get_current_pos(listener)


        #   получаем угол относительно целевой точки
        # Пересчитываем углы относительно курса
        goal_pos = [target_pos.pose.point.x, target_pos.pose.point.y, target_pos.pose.point.z]
        angle_goal_point = -angle_between(current_pos, goal_pos)
        angle_between_course_and_goal = angle_goal_point - current_course
        angle_between_course_and_goal = norm_angle(angle_between_course_and_goal)
        angle_vel = angle_between([0.0, 0.0, 0.0],velocity)
        angle_between_vel = angle_goal_point - angle_vel
        angle_between_vel = norm_angle(angle_between_vel)

        print("================\n"
              "course: %s\n"
              "angle_goal_point: %s \n"
              "between target %s \n"
              "vel: %s\n"
              "angle_between_vel %s" % (
              math.degrees(current_course),
              math.degrees(angle_goal_point),
              math.degrees(angle_between_course_and_goal),
              math.degrees(angle_vel),
              math.degrees(angle_between_vel)))
        vel_ang_pub.publish(pub_vel_angle())

        # Включаем планировщик когда придут данный о целевой точки
        if initFlag and len(sonar_data) > 0:

            # 2. Разбиваем данные с сонара на сектора,
            # а также поворачиваем сонар относительно целевой точки
            sectors = sonar_to_sectors(sonar_data)
            # 3. Получаем дистанцию до целевой точки
            target_dist = get_distance_to(current_pos, target_pos)

            # 4. Получаем угол и дистанцию на которую нужно отклониться
            dist = 0.0001
            if sectors:
                free_dir, dist = get_free_direct(np.asarray(sectors)[:, 0], target_dist, r_max)
            else:
                free_dir = 0.0

            # 5. Если угол 0, то летим к заданной точке
            if free_dir == 0.0 or state_interpolete:
               # Интерполируем точку
               # new_target = copy.deepcopy(target_pos)
               # new_lerp_point = lerp_point(current_pos,new_target, 0.5)

               # Шлём в ругулятор точку без изменений
               pub_goal.publish(target_pos)
               #  Рисуем маркер
               pub_marker.publish(setup_marker(target_pos, True))
               state_pub.publish(False)

            # 6. Если угол не 0, то поворачиваем на нужный угол
            else:
               #  Получаем новую точку с учётом смещения
               new_target = copy.deepcopy(target_pos)
               pos = rotate_vect(current_pos, new_target, free_dir, dist)
               # Интерполируем её

               # new_lerp_point = lerp_point(current_pos,pos, 0.5)
               # Шлём в ругулятор
               pub_goal.publish(pos)
               #  Рисуем маркер
               pub_marker.publish(setup_marker(pos, False))
               # print("rotate to", np.degrees(free_dir))
               state_pub.publish(True)
            # # 7. Выводим график для отладки
            # if(len(sectors) > 0):
            #     plt.plot(np.asarray(sectors,dtype=np.float32)[:,1],np.asarray(sectors,dtype=np.float32)[:,0])
            #     plt.xlabel(str(math.degrees(free_dir)) + "0 val : " + str(sectors[len(sectors) / 2]) + "vel angle: " + str(math.degrees(angle_between_vel)) + "target angl: " + str(math.degrees(angle_goal_point)))
            # plt.grid(True)
            # plt.pause(0.05)
            # plt.cla()

        rate.sleep()


if __name__ == '__main__':
    main()
