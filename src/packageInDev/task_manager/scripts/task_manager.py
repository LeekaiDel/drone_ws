#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Pose, Point
from drone_msgs.msg import GlobalTrajectory, Goal, DronePose
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
import math
import threading
import sys

class TaskManager():
    def __init__(self):
        self.height_of_takeoff = 1.0    # Высота взлета

        self.drone_is_takeoff = False   # Дрон взлетел?
        self.drone_is_land = False  # Дрон приземлился?
        self.allow_task_execution = False   # Разрешить выполнение задания
        self.recive_traj = False    # Получена траектория?

        self.drone_state = ExtendedState()
        self.curent_drone_pose = PoseStamped()
        self.curent_goal_traj = GlobalTrajectory()
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.drone_pose_cb, queue_size=10)
        rospy.Subscriber("/astar/trajectory", GlobalTrajectory, self.trajectory_cb, queue_size=10)
        rospy.Subscriber("/mavros/extended_state", ExtendedState, self.drone_state_cb, queue_size=10)
        
        self.goal_pub = rospy.Publisher("/goal_pose", Goal, queue_size=10)
        
        thread_input = threading.Thread(target=self.task_menu_cb, daemon=True)
        thread_input.start()
        thread_main = threading.Thread(target=self.main, daemon=True)
        thread_main.start()

    # Функция проверяет наличие нового задания и передает траекторию на регулятор
    def main(self):
        while True:
            if self.recive_traj == True:
                last_waypoint = PoseStamped()
                print("\nВыполнение задания...")
                for waypoint in self.curent_goal_traj.waypoints:
                    while round(math.sqrt((self.curent_drone_pose.pose.position.x - waypoint.pose.position.x)**2 + (self.curent_drone_pose.pose.position.y - waypoint.pose.position.y)**2), 1) > 0.5:
                        goal = Goal()
                        goal.pose.point.x = waypoint.pose.position.x
                        goal.pose.point.y = waypoint.pose.position.y
                        goal.pose.point.z = self.height_of_takeoff
                        if self.allow_task_execution:
                            self.goal_pub.publish(goal)
                self.recive_traj = False
                self.curent_goal_traj = GlobalTrajectory()
                print("Задание выполнено")

    # Функция обработки ввода/вывода команд в консоль
    def task_menu_cb(self):
        while True:
            print("\nПеречень команд:\n\t0. Стоп\n\t1. Взлет\n\t2. Посадка\n\t3. Выполнить задание\n\t ")
            command = input("Введите номер команды из перечня ->\t")
            if command == "exit":
                break

            elif command == "0":
                print("Команда: Стоп\n")
                self.allow_task_execution = False
                goal = Goal()
                goal.pose.point = self.curent_drone_pose.pose.position
                self.goal_pub.publish(goal)

            elif command == "1":
                print("Команда: Взлет\n")
                heigt = float(input("Введите высоту, на которую нужно взлететь ->\t"))
                
                self.height_of_takeoff = heigt
                if self.drone_is_land:
                    self.set_arm()
                    self.set_offboard_mode()
                goal = Goal()
                goal.pose.point.x = self.curent_drone_pose.pose.position.x
                goal.pose.point.y = self.curent_drone_pose.pose.position.y
                goal.pose.point.z = self.height_of_takeoff
                self.goal_pub.publish(goal)

            elif command == "2":
                print("Команда: Посадка\n")
                self.set_land()
                self.drone_is_land = True
            
            elif command == "3":
                print("Команда: Выполнить задание\n")
                cmd = input("Введите 0 или 1 - соответственно Запретить или Разрешить ->\t")
                if cmd == "0":
                    self.allow_task_execution = False
                    print("Выполнение задания ЗАПРЕЩЕНО")
                elif cmd == "1":
                    self.allow_task_execution = True
                    print("Выполнение задания РАЗРЕШЕНО")
            else:
                print("Не верная команда\n")
    
    # Устанавливаем OFFBOARD режим
    def set_offboard_mode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            response = flightModeService(custom_mode='OFFBOARD')
            return response.mode_sent
        except rospy.ServiceException as e:
            print("Offboard ошибка!")
            return False

    # Произвести запуск моторов
    def set_arm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            print("Запуск моторов...")
            armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            armService(True)
        except rospy.ServiceException as e:
            print("Ошибка запуска моторов!")

    # Произвести отключение моторов
    def set_disarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            print("Отключение моторов...")
            armService = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print("Ошибка отключения моторов!")

    # Производим посадку в текущей позиции
    def set_land(self):
        rospy.wait_for_service('mavros/cmd/land')
        try:
            print("Выполнение посадки...")
            landingService = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)
            info = landingService(0.0, 0.0, 0.0, 0.0, 0.0)
            # print(info)
        except:
            pass

    # Получаем данные о позиции дрона
    def drone_pose_cb(self, msg: PoseStamped):
        self.curent_drone_pose = msg
    
    # Получаем данные о текущем состоянии дрона
    def drone_state_cb(self, msg: ExtendedState):
        self.drone_state = msg
        if msg.landed_state == ExtendedState.LANDED_STATE_IN_AIR:
            self.drone_is_takeoff = True
            self.drone_is_land = False
        elif msg.landed_state == ExtendedState.LANDED_STATE_ON_GROUND:
            self.drone_is_takeoff = False
            self.drone_is_land = True


    # Получаем самую свежую траекторию
    def trajectory_cb(self, msg: GlobalTrajectory):
        if self.recive_traj == False:
            self.curent_goal_traj = msg
            self.recive_traj = True


if __name__ == "__main__":
    rospy.init_node("task_manager_node")
    TaskManager()
    rospy.spin()
    # rospy.destroy_node()
    # rospy.shutdown()
