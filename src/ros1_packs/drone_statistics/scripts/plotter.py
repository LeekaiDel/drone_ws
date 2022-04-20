#!/usr/bin/env python3
# coding=utf8

import math
import numpy as np
import rospy
import time
from std_msgs.msg import Float32MultiArray
from drone_msgs.msg import DronePose, Goal
from geometry_msgs.msg import Vector3, PoseStamped
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
import tf.transformations as t
import tf
import matplotlib.pyplot as plt  # creation plots
import copy

# ---------------------------------------------------------------------------------------------------------------------
# ---------- Глобальные переменные
use_gps = False
mavros_root = "/mavros"
drone_pos_topic = "/mavros/local_position/pose"
drone_pose = DronePose()  # Координаты дрона
prev_drone_pose = DronePose()
goal = DronePose()  # Целевая позиция дрона
cur_nav_vel = Vector3()  # Текущая скорость дрона
current_ctr = [0] * 4

lidar_topic = "/scan"
lidar_dist = [10.0, 0.0]  # min и мидиана

free_dist = 0.5
goal_point = Goal()
goal_point.pose.point.x = -30.0
goal_point.pose.point.y = 22.0
goal_point.pose.point.z = 2.0
goal_point.pose.course = 2.49


path = float()  # пройденный путь
received_goal = False


# ---------------------------------------------------------------------------------------------------------------------
# ---------- Функции подписки на топики

def goal_cb(data):
    """
    Функция считывания целевой точки.

    @param data: Целевая точка
    @type data: drone_reg.msg.Goal
    """
    global goal
    global received_goal
    goal = data.pose
    received_goal = True


def imu_cb(data):
    """
    Функция считывания данных инерциальной системы дрона.

    @param data: данные инерциальной ситсемы
    @type data: sensor_msgs.msg.Imu
    """
    global drone_pose
    drone_pose.course = get_yaw_from_quat([data.orientation.x, data.orientation.y,
                                           data.orientation.z, data.orientation.w])


def nav_vel_cb(data):
    """
    Функция считывания текущей скорости дрона.

    :param data: скорость
    :type data: geometry_msgs.msg.TwistStamped
    """
    global cur_nav_vel
    cur_nav_vel = data.twist.linear


def ctr_cb(data):
    """
    Функция сичтывает значения управляющих команд дрона.

    :param data: команды управления
    :type data: std_msgs.msg.Float32MultiArray
    """
    global current_ctr
    current_ctr = data.data


def set_nav(trans, rot):
    """
    Функция задания координат и кватерниона как текущих координат дрона.

    @param trans: координаты
    @param rot: кватернион
    """
    global drone_pose

    drone_pose.point.x = trans[0]
    drone_pose.point.y = trans[1]
    drone_pose.point.z = trans[2]
    drone_pose.course = get_yaw_from_quat(rot)


def set_nav_gps(pose):
    """
       Функция задания координат и кватерниона как текущих координат дрона.

       @param pose: координаты
       @:type pose: PoseStamped
       """
    global drone_pose, prev_drone_pose
    set_path()

    prev_drone_pose = copy.deepcopy(drone_pose);

    drone_pose.point.x = pose.pose.position.x
    drone_pose.point.y = pose.pose.position.y
    drone_pose.point.z = pose.pose.position.z


#    drone_pose.course = get_yaw_from_quat(pose.pose.orientation)

def set_path():
    global path, drone_pose, prev_drone_pose
    vec_pose = [prev_drone_pose.point.x - drone_pose.point.x,
                prev_drone_pose.point.y - drone_pose.point.y,
                prev_drone_pose.point.z - drone_pose.point.z]
    dt_pose = math.sqrt(vec_pose[0] ** 2 + vec_pose[1] ** 2 + vec_pose[2] ** 2)
    if dt_pose > 1:
        return
    path += dt_pose


def laser_scan_cb(data):
    global lidar_dist

    laser = data.ranges
    current_min = min(laser)  # find current min value
    lidar_dist[0] = min([lidar_dist[0], current_min])  # compare current min and absolute min values
    lidar_dist[1] = np.median(laser)


# ---------------------------------------------------------------------------------------------------------------------
# ---------- Вспомогательные функции

def get_yaw_from_quat(q):
    """
    Функция возвращает угол рысканья полученный из кватерниона.

    @param q: кватернион
    @type q: list
    @return: угол рысканья
    """
    return t.euler_from_quaternion(q, axes='sxyz')[2]

def get_dist_to(current_pose, target_pose):
    """
    :type current_pose: DronePose()
    :type target_pose: DronePose()
    :return:
    """
    vec_pose = [current_pose.point.x - target_pose.point.x,
                current_pose.point.y - target_pose.point.y,
                current_pose.point.z - target_pose.point.z]
    dist = math.sqrt(vec_pose[0] ** 2 + vec_pose[1] ** 2 + vec_pose[2] ** 2)
    return  dist

# ---------------------------------------------------------------------------------------------------------------------
# ---------- Главная функция

def main():
    """
    Основной цикл узла ROS.

    @return: код завершения программы
    """
    global drone_pose
    global goal
    global cur_nav_vel
    global current_ctr
    global path, lidar_dist
    global received_goal
    global goal_point, free_dist
    # Инициализируем узел ROS
    rospy.init_node("drone_reg_plotter")

    # Подписываемся на топики
    rospy.Subscriber("/goal_pose", Goal, goal_cb)
    rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, nav_vel_cb)
    rospy.Subscriber(mavros_root + "/local_position_ned/set_target/vel", Float32MultiArray, ctr_cb)
    rospy.Subscriber(lidar_topic, LaserScan, laser_scan_cb)
    pub_target = rospy.Publisher("/goal_pose", Goal, queue_size=10)


    if (use_gps):
        rospy.Subscriber(drone_pos_topic, PoseStamped, set_nav_gps)

    # Инициализация "слушателя" навигационных преобразований
    tf_listener = tf.TransformListener()

    # Инициализируем списки для построения графиков
    drone_x = list()
    drone_y = list()
    drone_z = list()
    drone_vx = list()
    drone_vy = list()
    drone_vz = list()
    goal_x = list()
    goal_y = list()
    goal_z = list()
    ctr_x = list()
    ctr_y = list()
    ctr_z = list()
    time_plot = list()
    lidar_min = list()
    lidar_av = list()

    old_ros_time = time.time()
    # Основной цикл
    rate = rospy.Rate(20)
    t_ = 0.0
    while not rospy.is_shutdown():
        # Сичтываем навигационные данные
        if not use_gps:
            try:
                (trans, rot) = tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                set_nav(trans, rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        # Рассчитываем время
        dt = time.time() - old_ros_time
        old_ros_time = time.time()
        # if t_ > 3.0:
        #     pub_target.publish(goal_point)
        # if received_goal:
        #     received_goal = False
        #     t_ = 0
        #     drone_x = list()
        #     drone_y = list()
        #     drone_z = list()
        #     drone_vx = list()
        #     drone_vy = list()
        #     drone_vz = list()
        #     goal_x = list()
        #     goal_y = list()
        #     goal_z = list()
        #     ctr_x = list()
        #     ctr_y = list()
        #     ctr_z = list()
        #     time = list()
        #     lidar_min = list()
        #     lidar_av = list()


        # Заполняем данные для графиков
        if get_dist_to(drone_pose, goal_point.pose) > free_dist:
            t_ += dt
            drone_x.append(drone_pose.point.x)
            drone_y.append(drone_pose.point.y)
            drone_z.append(drone_pose.point.z)
            drone_vx.append(cur_nav_vel.x)
            drone_vy.append(cur_nav_vel.y)
            drone_vz.append(cur_nav_vel.z)
            goal_x.append(goal.point.x)
            goal_y.append(goal.point.y)
            goal_z.append(goal.point.z)
            # NED to ENU
            ctr_x.append(current_ctr[1])  # y
            ctr_y.append(current_ctr[0])  # x
            ctr_z.append(-current_ctr[2])  # -z
            lidar_min.append(lidar_dist[0])
            lidar_av.append(lidar_dist[1])
            time_plot.append(t_)

        else:
            print('t %s' % t_)
            print('path: %s' % path)
            print("lidar_dist:", lidar_dist)
            print("dist to point:", get_dist_to(drone_pose, goal_point.pose))

        # Рисуем графики
        ax1 = plt.subplot2grid((2, 3), (0, 0))
        ax1.plot(time_plot, drone_x, label='drone x')
        ax1.plot(time_plot, goal_x, label='goal x')
        ax1.set_title('Coords: drone X and goal X')
        ax1.set_ylabel('x, m')
        ax1.set_xlabel('t, s')
        ax1.legend()
        ax1.grid()

        ax2 = plt.subplot2grid((2, 3), (0, 1))
        ax2.plot(time_plot, drone_y, label='drone y')
        ax2.plot(time_plot, goal_y, label='goal y')
        ax2.set_title('Coords: drone Y and goal Y')
        ax2.set_ylabel('y, m')
        ax2.set_xlabel('t, s')
        ax2.legend()
        ax2.grid()

        ax3 = plt.subplot2grid((2, 3), (0, 2))
        ax3.plot(time_plot, drone_z, label='drone z')
        ax3.plot(time_plot, goal_z, label='goal z')
        ax3.set_title('Coords: drone Z and goal Z')
        ax3.set_ylabel('z, m')
        ax3.set_xlabel('t, s')
        ax3.legend()
        ax3.grid()

        ax4 = plt.subplot2grid((2, 3), (1, 0))
        ax4.plot(drone_x, drone_y, label='drone')
        ax4.plot(goal_x, goal_y, label='goal')
        ax4.set_title('trajectory XY')
        ax4.set_xlabel('x, m')
        ax4.set_ylabel('y, m')
        ax4.legend()
        ax4.grid()

        # ax8 = plt.subplot2grid((2, 3), (1, 1))
        # # ax8.plot(time, lidar_min, label='distance')
        # ax8.plot(time, lidar_av, label='median')
        # ax8.set_title('lidar')
        # ax8.set_xlabel('t, s')
        # ax8.set_ylabel('dist, m')
        # ax8.legend()
        # ax8.grid()

        # for i, ax in enumerate(plt.gcf().axes):
        #     ax.text(0.5, 0.5, "ax%d" % (i + 1), va="center", ha="center")

        plt.pause(1.0 / 50.0)

        rate.sleep()
    return 0


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
