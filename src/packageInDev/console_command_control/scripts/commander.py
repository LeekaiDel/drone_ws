#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Pose, Point
from drone_msgs.msg import GlobalTrajectory, Goal, DronePose
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
import math
import threading
import os

"""
Атрибуты статуса БЛА:
    Рабочая высота: 0.5
    Режим управления: Offboard/Manual
    Статус моторов: Запущены/Отключены
    Выполнение задания: В процессе/Приостановлено/Окончено/Прервано
"""

class TaskManager():
    def __init__(self):
        self.height_of_takeoff = 0.5        # Высота взлета

        self.drone_is_takeoff = False       # Дрон взлетел?
        self.drone_is_land = False          # Дрон приземлился?
        self.allow_task_execution = False   # Разрешить выполнение задания

        self.drone_state = ExtendedState()
        self.curent_drone_pose = PoseStamped()
        self.curent_goal_traj = GlobalTrajectory()
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.drone_pose_cb, queue_size=10)
        rospy.Subscriber("/mavros/extended_state", ExtendedState, self.drone_state_cb, queue_size=10)

        # self.goal_pub = rospy.Publisher("/goal_pose", Goal, queue_size=10)
        
        # thread_main = threading.Thread(target=self.main, daemon=True)
        # thread_main.start()

        while True:
            os.system("clear")
            print("\
Атрибуты статуса БЛА:\n\n\
    Рабочая высота: 0.5\n\
    Режим управления: Offboard/Manual\n\
    Статус моторов: Запущены/Отключены\n\
    Выполнение задания: В процессе/Приостановлено/Окончено/Прервано\n\n\
\
Перечень команд:\n\n\
    0. Стоп\n\
    1. Offboard\n\
    2. Взлет\n\
    3. Посадка\n\
    +. Выключить моторы\n\
    R. Возврат домой\n\
    ET. Выполнение задания\n\
    SH. Установить рабочую высоты\n\
    exit. Выход\n"
                )
            command = input("Введите идентификатор команды из перечня -> ")
            
            if command == 'exit' or command == 'e':
                exit()

            elif command == '0':
                print('Команда: Стоп\n')
                self.allow_task_execution = False
                goal = Goal()
                goal.pose.point = self.curent_drone_pose.pose.position
                # self.goal_pub.publish(goal)

            elif command == '1': #TODO: Сделать переключение режимов
                pass

            elif command == '2':
                print('Команда: Взлет\n')
                heigt = float(input('Введите высоту, на которую нужно взлететь -> '))
                self.height_of_takeoff = heigt
                if self.drone_is_land:
                    self.set_arm()
                    self.set_offboard_mode()
                # goal = Goal()
                # goal.pose.point.x = self.curent_drone_pose.pose.position.x
                # goal.pose.point.y = self.curent_drone_pose.pose.position.y
                # goal.pose.point.z = self.height_of_takeoff
                # self.goal_pub.publish(goal)

            elif command == '2':
                print('Команда: Посадка\n')
                self.set_land()
                self.drone_is_land = True
            
            # elif command == '3':
            #     print('Команда: Выполнить задание\n')
            #     cmd = input('Введите 0 или 1 - соответственно Запретить или Разрешить -> ')
            #     if cmd == '0':
            #         self.allow_task_execution = False
            #         print('Выполнение задания ЗАПРЕЩЕНО')
            #     elif cmd == '1':
            #         self.allow_task_execution = True
            #         print('Выполнение задания РАЗРЕШЕНО')

            # elif command == '4':
            #     print('Команда: Выключить моторы\n')
            #     self.set_disarm()
            
            # elif command == '5':
            #     print('Команда: Назад домой\n')
            #     self.return_to_home()

            else:
                print('Не верная команда\n')
    
    # Возврат домой
    def return_to_home(self):
        return_path = list()
        return_path.extend(self.completed_path)
        return_path.reverse()
        # print("Input:" + str(self.completed_path))
        for goal_point in return_path:
            # print(round(math.sqrt((goal_point.pose.point.x - self.curent_drone_pose.pose.position.x)**2 + (goal_point.pose.point.y - self.curent_drone_pose.pose.position.y)**2), 1))
            while round(math.sqrt((goal_point.pose.point.x - self.curent_drone_pose.pose.position.x)**2 + (goal_point.pose.point.y - self.curent_drone_pose.pose.position.y)**2), 1) > self.delta_r:
                self.goal_pub.publish(goal_point)
        self.completed_path.clear()
        # print("Output:" + str(len(self.completed_path)))
        print("Задание выполнено")

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


if __name__ == "__main__":
    rospy.init_node("task_manager_node")
    TaskManager()
    rospy.spin()
