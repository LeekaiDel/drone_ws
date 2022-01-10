#!/usr/bin/env python
# coding=utf8

import rospy
import tf
import copy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
from visualization_msgs.msg import *
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse

from drone_msgs.msg import Goal
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PoseStamped, TwistStamped

from dynamic_reconfigure.server import Server
from unstable_planner.cfg import unstableCfgConfig
from params_saver import YamlParams
import os.path

# флаг входных данных
# True - point_cloud2
# False - lidar
input_point2 = False


# Параметры планировщика
r_view = 1.0                            # расстояние срабатывания планировщика
rc_min = 0.5                            # минимальное расстояне до препятствия
view_angle = 240                        # угол обзора планировщика
velocity_gain = 0.1                     # скорость дрона, до которой она считается равной нулю
k_buff = 2.0					# коэффициент буффиркационного параметра
beta_max = k_buff * (r_view - rc_min)        # максимально возможная бета
dist_offset = 1.0                       # дистанция на которую ставится виртуальная точка от дрона

# топики лазерскан и поинтклауд
lidar_topic = "/scan"
octomap_topic = "/scan_matched_points2"

goal_sub_topic = "/goal_pose"
goal_pub_topic = "/goal_pose_to_reg"
pose_topic = "/mavros/local_position/pose"
velocity_topic = "/mavros/local_position/velocity"
marker_topic = "/marker_target_point"

lp = lg.LaserProjection()

target_pose = Goal()                # целевая точка
virtual_target_pose = Goal()        # виртуальная ЦТ

current_pose = [0.0, 0.0, 0.0]      # текущая позиция дрона
current_course = 0.0                # текущий курс дрона
vec_to_goal = [0.0, 0.0, 0.0]       # вектор между дроном и ЦТ
vel_vec_2d = [0.0, 0.0]             # вектор скорости в плоскости

min_dist = 999.0                    # расстояние до ближайщего препятствия
min_dist_index = 0                  # угол по лидару до ближ. преп.
angle_rad = 0.0                     # угол на который надо повернуть

init_server = False
active_flag = True				# состояние, включен ли планировщик
yaml_path = os.path.dirname(__file__)+'/../../cfg/params.yaml'
cfg_srv = None
pub_goal = None

##################################################################################
#                       Функция считывания параметров планировщика
##################################################################################
def cfg_callback(config, level):

    global r_view, rc_min, view_angle, velocity_gain, init_server, pps, beta_max, k_buff, dist_offset, active_flag

    # Параметры регулятора
    r_view = config["r_view"]
    rc_min = config["rc_min"]
    view_angle = config["view_angle"]
    velocity_gain = config["velocity_gain"]
    dist_offset = config["dist_offset"]
    k_buff = config["k_buff"]

    if init_server:
        # opening pid params file
        pps.params_open()
        print("open from cfg callback")

        # setting values
        pps.params_set('r_view', config["r_view"])
        pps.params_set('rc_min', config["rc_min"])
        pps.params_set('view_angle', config["view_angle"])
        pps.params_set('velocity_gain', config["velocity_gain"])
        pps.params_set('dist_offset', config["dist_offset"])
        pps.params_set('k_buff', config["k_buff"])

        # saving to file
        pps.params_save()
        print("save from cfg callback")

        active_flag = config["run"]
    init_server = True
    beta_max = k_buff * (r_view - rc_min)  # максимально возможная бета
    return config


##################################################################################
#                       Функция считывания параметров планировщика
##################################################################################
def set_server_value(cfg_srv):
    global r_view, rc_min, view_angle, velocity_gain, beta_max, k_buff, dist_offset, active_flag
    global init_server, pps

    pps.params_open()
    print ("open from set server value")

    r_view = pps.params_get('r_view')
    rc_min = pps.params_get('rc_min')
    view_angle = pps.params_get('view_angle')
    velocity_gain = pps.params_get('velocity_gain')
    k_buff = pps.params_get('k_buff')
    dist_offset = pps.params_get('dist_offset')

    cfg_srv.update_configuration({'r_view': pps.params_get('r_view'),
                                  'rc_min': pps.params_get('rc_min'),
                                  'view_angle': pps.params_get('view_angle'),
                                  'velocity_gain': pps.params_get('velocity_gain'),
                                  'k_buff': pps.params_get('k_buff'),
                                  'dist_offset': pps.params_get('dist_offset'),
                                  'run': active_flag})
    pps.params_save()
    beta_max = k_buff * (r_view - rc_min)  # максимально возможная бета
    print ("save from set server value")


##################################################################################
#                       Функция получения текущих координат
##################################################################################
def callbackCurrentPose(data):
    """
    :type data: PoseStamped
    :param data:
    :return:
    """
    global current_pose, current_course

    current_pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]

    # Получаем текущий курс дрона
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion((data.pose.orientation.x,
                                                                   data.pose.orientation.y,
                                                                   data.pose.orientation.z,
                                                                   data.pose.orientation.w))
    current_course = yaw


##################################################################################
#                   Функция получения текущей скорости в плоскости
##################################################################################
def callbackVelocity(data):
    """
    :type data: TwistStamped
    :param data:
    :return:
    """
    global vel_vec_2d
    vel_vec_2d = [data.twist.linear.x, data.twist.linear.y]


##################################################################################
#                       Функция получения координат цели
##################################################################################
def callbackTargetPose(data):
    """
    :type data: Goal
    :param data:
    :return:
    """
    global target_pose, active_flag, pub_goal
    target_pose = data
    if active_flag is False and pub_goal is not None:
        pub_goal.publish(data)


##################################################################################
#            Функция получения минимального расстояния и угла из LaserScan
##################################################################################
def scan_cb(msg):
    global min_dist, min_dist_index
    min_dist = min(msg.ranges)
    min_dist_index = msg.ranges.index(min_dist)


##################################################################################
#                       Функция получения облака точек из Octomap
##################################################################################
# не написана, тут надо находить ближайшую точку
def octomap_cb(msg):
    global point_list
    # конвертируем PointCloud2 в List
    # del point_list[:]
    point_list = pc2.read_points_list(msg)


##################################################################################
#          Функция получения угла поворота от расстояния до препятствия
##################################################################################
def get_angle(dist):
    global beta_max, r_view
    # выдаёт угол в градусах от 0 до 90(вру:) на который надо повернуть
    beta = abs(dist - r_view) - (dist - r_view)
    # deg = (beta * 90) / beta_max
    rad = (beta * np.pi) / (2 * beta_max)
    return rad


##################################################################################
#                           Функция настройки маркера
##################################################################################
def setup_marker(pose, free_flag):
    """
    Настройка маркера для отображения в rviz
    :type pose: Point
    :type free_flag: Bool
    :param pose:
    :param free_flag:
    :return:
    """
    marker = Marker()
    marker.header.frame_id = "map"
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
    marker.color.r = 0.0 if free_flag else 1.0
    marker.color.g = 1.0 if free_flag else 0.0
    marker.color.b = 0.0
    marker.color.a = 0.8
    marker.pose.position.x = pose.pose.point.x
    marker.pose.position.y = pose.pose.point.y
    marker.pose.position.z = pose.pose.point.z

    return marker

def set_active_srv(req):
    """
    Change state of planner
    """
    global active_flag, cfg_srv
    active_flag = req.data
    print("Change state:", active_flag)
    resp = SetBoolResponse()
    resp.success = True

    if cfg_srv is not None:
        # write in dyn. rec.
        cfg_srv.update_configuration({'run': active_flag})

    return resp

##################################################################################
#                          Функция поворота целевой точки
##################################################################################
def rotate_vect(dr_pose, goal, angle, dist_offset=None):
    """
    Поворачиваем b относительно a на угол rot
    :type dr_pose: list
    :type goal: Goal
    :type angle: float
    :param dr_pose:
    :param goal:
    :param angle:
    :return: возвращаем точку повёрнутую на нужный угол

    """
    # матрица поворота
    rotate = np.array([[np.cos(angle), -np.sin(angle)],
                       [np.sin(angle), np.cos(angle)]])
    # разница координат дрона и ЦТ
    pos = np.array([[goal.pose.point.x - dr_pose[0]],
                    [goal.pose.point.y - dr_pose[1]]])
    try:
        # повернутый вектор
        val = np.dot(rotate, pos)
        # normVec = val / np.linalg.norm(val) * dist

        # получаем координаты новой ЦТ в ГК
        if dist_offset != None:
            # смещаем точки
            dist = np.linalg.norm(val)
            # если добрались до конечной, то отключаем интерполяцию
            # if dist > dist_offset:
            # Умножаем нормализованный вектор на скаляр
            val = val / dist * dist_offset

        val[0] += dr_pose[0]
        val[1] += dr_pose[1]
    except:
        val = [goal.pose.point.x, goal.pose.point.y]

    goal.pose.point.x = val[0]
    goal.pose.point.y = val[1]

    return goal


# нормализация угла в пределах -180:180 градусов
def norm_angle_deg(ang):
    while ang < -180:
        ang += 360
    while ang > 180:
        ang -= 360
    return ang


# нормализация угла в пределах -Pi:Pi радиан
def norm_angle_rad(ang):
    while ang < -np.pi:
        ang += 2*np.pi
    while ang > np.pi:
        ang -= 2*np.pi
    return ang


##################################################################################
#                               Главная функция
##################################################################################
def main():
    global angle_rad, virtual_target_pose, beta_max, pps, yaml_path, input_point2, cfg_srv, pub_goal
    global current_pose, vec_to_goal
    rospy.init_node('unstable_newest', anonymous=True)
    rate = rospy.Rate(10)

    yaml_path = rospy.get_param("~yaml_path", yaml_path)
    input_point2 = rospy.get_param("~use_point2", input_point2)
    pps = YamlParams(yaml_path)

    print("yaml_path: ", yaml_path)
    # init dynamic reconfigure server
    cfg_srv = Server(unstableCfgConfig, cfg_callback)
    # переопределение значений сервера из файла
    set_server_value(cfg_srv)
    # переопределение значений файла из сервера

    print("init new unstable planner")
    if input_point2:
        # подписываемся на Octomap
        rospy.Subscriber(octomap_topic, PointCloud2, octomap_cb)
    else:
        # подписываемся на скан лидара
        rospy.Subscriber(lidar_topic, LaserScan, scan_cb)

    # подписываемся на текущую целевую точку
    rospy.Subscriber(goal_sub_topic, Goal, callbackTargetPose)
    # подписываемся на текущую позицию дрона
    rospy.Subscriber(pose_topic, PoseStamped, callbackCurrentPose)
    # подписываемся на текущую скорость дрона
    rospy.Subscriber(velocity_topic, TwistStamped, callbackVelocity)

    # паблишер ЦТ в регулятор
    pub_goal = rospy.Publisher(goal_pub_topic, Goal, queue_size=10)
    # паблишер маркера
    pub_marker = rospy.Publisher(marker_topic, Marker, queue_size=10)
    # паблишер состояния планировщика
    status_pub = rospy.Publisher("/drone/local_planner/unstable/status", Bool, queue_size=10)
    active_pub = rospy.Publisher("/drone/local_planner/unstable/active", Bool, queue_size=10)
    # сервис для отключения планировщика
    rospy.Service("drone/local_planner/unstable/set_active", SetBool, set_active_srv)

    while not rospy.is_shutdown():

        active_pub.publish(active_flag)
        if not active_flag:
            status_pub.publish(False)
            rate.sleep()
            continue

        # вычисляем вектор между текущим положением дрона и целью
        vec_to_goal = [target_pose.pose.point.x - current_pose[0],
                       target_pose.pose.point.y - current_pose[1],
                       target_pose.pose.point.z - current_pose[2]]
        ang_ct = np.arctan2(vec_to_goal[1], vec_to_goal[0])
        # print np.rad2deg(ang_ct)

        # проверяем что препятствие в зоне радиуса R
        if min_dist < r_view:
            # возвращает angle_deg угол на который надо повернуть вектор скорости
            angle_rad = get_angle(min_dist)

            # направление от дрона на ЦТ в ГК
            # ang_ct = np.arctan2(vec_to_goal[1], vec_to_goal[0])

            # ang_drone_obs - угол между дроном и препятствием в ГК
            ang_drone_obs = min_dist_index + np.rad2deg(current_course)-180
            # нормализуем угол
            ang_drone_obs = norm_angle_deg(ang_drone_obs)

            # ang_vel - угол ВС в ГК
            if np.sqrt(vel_vec_2d[0]**2 + vel_vec_2d[1]**2) < velocity_gain:
                # если ВС равен нулю -  считаем его равным направлению на ЦТ
                ang_vel_rad = ang_ct
            else:
                # иначе - вычисляем направления ВС в ГК
                ang_vel_rad = np.arctan2(vel_vec_2d[1], vel_vec_2d[0])

            # ang_vel_obs -  угол между ВС и препятствием в ГК
            ang_vel_obs = ang_drone_obs - np.rad2deg(ang_vel_rad)
            # нормализуем угол
            ang_vel_obs = norm_angle_deg(ang_vel_obs)

            # если препятствие находится в области (+-половина от угла обзора) градусов от ВС
            if abs(ang_vel_obs) < view_angle/2:
                # копируем накой-то ЦТ
                target_copy = copy.deepcopy(target_pose)

                # поворачиваем ЦТ относительно текущей позиции дрона
                # препятствие слева
                if ang_vel_obs >= 0:
                    # вычисляем угол поворота целевой точки
                    rot_ang = norm_angle_rad(-angle_rad + ang_vel_rad - ang_ct)
                    virtual_target_pose = rotate_vect(current_pose, target_copy, rot_ang,dist_offset=dist_offset)

                # препятствие справа
                elif ang_vel_obs < 0:
                    # вычисляем угол поворота целевой точки
                    rot_ang = norm_angle_rad(angle_rad + ang_vel_rad - ang_ct)
                    virtual_target_pose = rotate_vect(current_pose, target_copy, rot_ang, dist_offset=dist_offset)

                # публикуем повернутую точку
                pub_goal.publish(virtual_target_pose)
                pub_marker.publish(setup_marker(virtual_target_pose, False))
                status_pub.publish(True)
            # иначе, если препятствие сзади
            else:
                # не вертим ВЦТ
                pub_goal.publish(target_pose)
                pub_marker.publish(setup_marker(target_pose, True))
                status_pub.publish(False)

        # иначе, если препятствие не в радиусе R
        else:
            # не вертим ВЦТ
            pub_goal.publish(target_pose)
            pub_marker.publish(setup_marker(target_pose, True))
            status_pub.publish(False)

        rate.sleep()


if __name__ == '__main__':
    main()

