#!/usr/bin/env python3
import rospy
import curses
from geometry_msgs.msg import PoseStamped #, Pose, Point
from drone_msgs.msg import GlobalTrajectory, DronePose, CmdManagerControlCmd, TaskCmd
from mavros_msgs.msg import ExtendedState
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
# import math
import threading
import os
import time


class ConsoleCommander():
    def __init__(self):
        self.height_of_takeoff = 0.5        # Высота взлета

        self.mode_control = False
        
        self.drone_is_takeoff = False       # Дрон взлетел?
        self.drone_is_land = False          # Дрон приземлился?
        self.allow_task_execution = False   # Разрешить выполнение задания?
        self.listen_command = True          # Слушать команды меню?

        self.input_command = None           # Переменная для хранения последней введенной команды

        self.stdscr = curses.initscr()      # Создаем объект для окна приложения
        # curses.noecho()                   # Запретить вывод вводимых данных 
        self.stdscr.keypad(True)        

        self.cmd = None
        self.drone_state = ExtendedState()
        self.current_drone_pose = PoseStamped()
        self.current_goal_traj = GlobalTrajectory()
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.drone_pose_cb, queue_size=10)
        rospy.Subscriber("/mavros/extended_state", ExtendedState, self.drone_state_cb, queue_size=10)

        self.task_pub = rospy.Publisher("/task_manager/task", TaskCmd, queue_size=10)

        self.stdscr.clear()
        self.win_teltemerty = self.stdscr.subwin(20, 100, 0, 0)     #(кол-во строк, кол-во столбцов, начало по y, начало по x)
        self.win_input = self.stdscr.subwin(1, 100, 21, 0)          #(кол-во строк, кол-во столбцов, начало по y, начало по x)
        
        self.thread_func_of_input = threading.Thread(target=self.func_of_input, daemon=True)
        self.thread_func_of_input.start()

        while not rospy.is_shutdown():
            try: 
                self.win_teltemerty.clear()
                self.win_teltemerty.addstr(0, 0, 'Атрибуты статуса БЛА:') 
                self.win_teltemerty.addstr(1, 5, f"Рабочая высота: {self.height_of_takeoff}")
                self.win_teltemerty.addstr(2, 5, f"Режим управления: {self.mode_control}")      #TODO: добавить отображение режима управления
                self.win_teltemerty.addstr(3, 5, f"Статус моторов:")
                self.win_teltemerty.addstr(4, 5, f"Выполнение задания:")
                self.win_teltemerty.addstr(5, 5, f"")
                self.win_teltemerty.addstr(6, 0, f"Локальная позиция по X: {round(self.current_drone_pose.pose.position.x, 2)}")
                self.win_teltemerty.addstr(7, 0, f"Локальная позиция по Y: {round(self.current_drone_pose.pose.position.y, 2)}")
                self.win_teltemerty.addstr(8, 0, f"Локальная позиция по Z: {round(self.current_drone_pose.pose.position.z, 2)}")
                self.win_teltemerty.addstr(9, 5, "")
                self.win_teltemerty.addstr(10, 0, "Перечень команд:")
                self.win_teltemerty.addstr(11, 5, "0. Выключить моторы")
                self.win_teltemerty.addstr(12, 5, "1. Запуск моторов и активация Offboard")
                self.win_teltemerty.addstr(13, 5, "2. Посадка")
                self.win_teltemerty.addstr(14, 5, "3. Взлет")
                self.win_teltemerty.addstr(15, 5, "+. Стоп")
                self.win_teltemerty.addstr(16, 5, "R. Возврат домой")
                self.win_teltemerty.addstr(17, 5, "ET. Выполнение задания")
                self.win_teltemerty.addstr(18, 5, "SH. Установить рабочую высоту")
                self.win_teltemerty.addstr(19, 5, "exit. Выход")
                self.win_teltemerty.refresh()
                self.win_input.refresh()        # Переводим курсор на окно ввода
            except curses.error: 
                pass


            if self.cmd == b'e' or self.cmd == b'exit':
                curses.endwin()
                exit()


            # Выключить моторы
            elif self.cmd == b'0':
                # print('Команда: Выключить моторы\n')
                self.set_disarm()


            # Включение Offboard и запуск моторов
            elif self.cmd == b'1':
                self.set_arm()
                self.set_offboard_mode()


            # Посадка
            elif self.cmd == b'2':
                task_command = TaskCmd()
                task_command.header.stamp = rospy.Time().now()
                task_command.type = TaskCmd.LAND
                self.task_pub.publish(task_command)


            # Взлет
            elif self.cmd == b'3':  #TODO: Добавить проверку на букву
                # print('Команда: Взлет\n')
                # hgt = input('Введите высоту, на которую нужно взлететь -> ')
                self.win_input.clear()
                self.win_input.addstr(0, 0, "Введите высоту, на которую нужно взлететь --> ")
                self.win_input.refresh()
                hgt = self.win_input.getstr(0, 46)
                self.win_input.clear()
                if hgt != b'':
                    self.height_of_takeoff = float(hgt)
                
                goal_pose = PoseStamped()
                goal_pose.pose.position.x = self.current_drone_pose.pose.position.x
                goal_pose.pose.position.y = self.current_drone_pose.pose.position.y
                goal_pose.pose.position.z = self.height_of_takeoff
                
                task_command = TaskCmd()
                task_command.header.stamp = rospy.Time().now()
                task_command.type = TaskCmd.TAKEOFF
                task_command.coordinates.append(goal_pose)

                self.task_pub.publish(task_command)


            # Остановка
            elif self.cmd == b'+':
                task_command = TaskCmd()
                task_command.header.stamp = rospy.Time().now()
                task_command.type = TaskCmd.STOP
                self.task_pub.publish(task_command)


            # Возврат по траектории в начальную точку
            elif self.cmd == b'R': 
                task_command = TaskCmd()
                task_command.header.stamp = rospy.Time().now()
                task_command.type = TaskCmd.RETURN
                self.task_pub.publish(task_command)
            

            #TODO: Запуск выполнения задания 
            elif self.cmd == b'ET':
                # print('Команда: Выполнить задание\n')
                # cmd = input('Введите 0 или 1 - соответственно Запретить или Разрешить -> ')
                # if cmd == b'0':
                #     self.allow_task_execution = False
                #     print('Выполнение задания ЗАПРЕЩЕНО')
                # elif cmd == b'1':
                #     self.allow_task_execution = True
                #     print('Выполнение задания РАЗРЕШЕНО')
                pass
            

            # Установка рабочей высоты
            elif self.cmd == b'SH': 
                # print('Команда: Установить рабочую высоту\n')
                self.win_input.clear()
                self.win_input.addstr(0, 0, "Введите рабочую высоту --> ")
                self.win_input.refresh()
                hgt = self.win_input.getstr(0, 27)
                self.win_input.clear()
                if hgt != b'':
                    self.height_of_takeoff = float(hgt)
            
            self.listen_command = True  # Возобновляем слушание команд меню
            self.cmd = None
            time.sleep(0.1)


    def func_of_input(self):
        while True:
            if self.listen_command:
                self.win_input.clear()
                self.win_input.addstr(0, 0, "Введите команду --> ")
                self.win_input.refresh()
                self.cmd = self.win_input.getstr(0, 20)
                self.listen_command = False # Останавливаем слушание команд меню


    # Устанавливаем OFFBOARD режим
    def set_offboard_mode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            response = flightModeService(custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print("Offboard ошибка!")


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


    # Получаем данные о позиции дрона
    def drone_pose_cb(self, msg: PoseStamped):
        self.current_drone_pose = msg
    

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
    rospy.init_node("console_commander_node")
    ConsoleCommander()
    rospy.spin()
