#!/usr/bin/env python3

import rospy
from copy import deepcopy
from std_msgs.msg import Float32
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Pose, Point, TwistStamped
from drone_msgs.msg import GlobalTrajectory, Goal, DronePose, TaskCmd, CmdManagerControlCmd
from mavros_msgs.msg import State, ExtendedState 
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from astar_planner.srv import GetTrajectory
import math
import threading
import sys
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class CommandManager:

    def __init__(self):
        # params:
        self.delta_r = 0.4                                  # Дельта окрестность достижения точки
        self.use_global_planner = True
        # variables:
        self.working_hight = 1.0                            # Рабочая высота
        self.current_drone_pose = PoseStamped()             # Текущая позиция дрона
        self.current_velocity = TwistStamped()              # Текущая скорость дрона
        self.completed_path = list()                        # Пройденный маршрут
        self.current_cmd =  TaskCmd()                       # Текущее задание
        self.control_cmd = CmdManagerControlCmd.WAITING     # Команда управления менеджером
        self.r_vel = None                                   # Модуль вектора скорости дрона в плоскости
        self.command_stack = list()                         # Это стек, в который поочередно помещаются пришедшие команды
        # flags:
        # self.get_new_cmd = False
        # self.stop_command = False
        # ros initialisations:
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.drone_pose_cb, queue_size=10)
        rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self.local_vel_cb, queue_size=10)
        rospy.Subscriber("/command_manager/command", TaskCmd, self.task_cmd_cb, queue_size=10)
        rospy.Subscriber("/command_manager/control_cmd", CmdManagerControlCmd, self.control_cmd_cb, queue_size=10)
        self.task_status_pub = rospy.Publisher("/command_manager/task_status", Goal, queue_size=10)
        self.goal_pub = rospy.Publisher("/goal_pose", Goal, queue_size=10)
        # thread_main = threading.Thread(target=self.main, daemon=True)
        # thread_main.start()
        # Функция проверяет наличие нового задания и передает траекторию на регулятор
        while not rospy.is_shutdown():
            # print(self.control_cmd)
            # print(self.command_stack)
            if self.control_cmd != CmdManagerControlCmd.WAITING:
                if self.command_stack:
                    #and self.control_cmd == CmdManagerControlCmd.START
                    for command in self.command_stack:
                            self.current_cmd = deepcopy(command)
                            while self.control_cmd == CmdManagerControlCmd.WAITING:
                                pass
                            # Взлет
                            if command.type == TaskCmd.TAKEOFF:
                                print("command takeoff:")
                                if command.coordinates:
                                    if self.takeoff_cmd(coordinate = command.coordinates[0]):
                                        print("command comleted")
                                    else:
                                        print("command break")
                                        self.control_cmd = CmdManagerControlCmd.WAITING
                                else:
                                    print("command ERROR")

                            # Полет в точку
                            elif command.type == TaskCmd.GO_TO_POINT:
                                print("command go_to_point:")
                                if command.coordinates:
                                    if self.go_to_point_cmd(command.coordinates[0]):
                                        print("command completed")
                                    else:
                                        print("command break")
                                        self.control_cmd = CmdManagerControlCmd.WAITING
                                else:
                                    print("command ERROR")

                            # Полет по траектории
                            elif command.type == TaskCmd.GO_ALONG_TRAJECTORY:
                                print("command go_along_traj_cmd:")
                                if self.go_along_trajectory_cmd(coordinates=command.coordinates):
                                    print("command completed")
                                else:
                                    print("command break")
                                    self.control_cmd = CmdManagerControlCmd.WAITING

                            # Стоп
                            elif command.type == TaskCmd.STOP:
                                print("command stop:") 
                                if self.stop_cmd():
                                    print("command completed")
                                else:
                                    print("command break")
                                    self.control_cmd = CmdManagerControlCmd.WAITING

                            # Возврат
                            elif command.type == TaskCmd.RETURN:
                                pass

                            # Посадка
                            elif command.type == TaskCmd.LAND:
                                print("command land:") 
                                if self.land_cmd():
                                    print("command completed")
                                else:
                                    print("command break")
                                    self.control_cmd = CmdManagerControlCmd.WAITING

                            # self.command_stack.remove(command)
                    self.command_stack.clear()
                else:
                    self.control_cmd = CmdManagerControlCmd.WAITING

    # Получаем данные о позиции дрона
    def drone_pose_cb(self, msg: PoseStamped):
        self.current_drone_pose = msg


    # Получаем самую свежую траекторию
    def task_cmd_cb(self, msg: TaskCmd):
        print("get new task")
        # if msg.priority >= self.current_cmd.priority:
        #     self.control_cmd = CmdManagerControlCmd.BREAK
        #     self.command_stack.clear()
        self.command_stack.append(msg)
        print("command stack has a " + str(len(self.command_stack)) + " commands")
        print(self.command_stack)
        # self.current_task = msg
        # self.get_new_cmd = True
    

    # Получаем команды управления менеджером
    def control_cmd_cb(self, msg: CmdManagerControlCmd):
        self.control_cmd = msg.control_cmd


    # Получаем локальную скорость дрона
    def local_vel_cb(self, msg: TwistStamped):
        self.current_velocity = msg
        self.r_vel = math.sqrt(pow(self.current_velocity.twist.linear.x, 2) + pow(self.current_velocity.twist.linear.y, 2))
        # print(self.r_vel)


    # Функция проверяет достигнута ли целевая точка по условию дельта-окрестности
    def reaching_delta_r(self, current_pose: PoseStamped, goal_pose: Goal, delta_r) -> bool:
        return round(math.sqrt((goal_pose.pose.point.x - current_pose.pose.position.x)**2 + (goal_pose.pose.point.y - current_pose.pose.position.y)**2) + (goal_pose.pose.point.z - current_pose.pose.position.z)**2, 1) >= delta_r


    # Функция вызова запроса к А*
    def call_a_star(self, goal_position) -> list: 
        print("waiting astar server...")
        rospy.wait_for_service('/astar/get_trajectory')
        print("astar server connected")
        hight = Float32()
        hight.data = self.working_hight
        try:
            get_path_service = rospy.ServiceProxy('/astar/get_trajectory', GetTrajectory)
            resp = get_path_service(goal_position, hight)
            return resp.path
        except rospy.ServiceException as e:
            print("service AStar call failed: %s"%e)


    # FIXME: Команда взлет
    def takeoff_cmd(self, coordinate: PoseStamped) -> bool:
        goal = Goal()
        goal.pose.point.x = self.current_drone_pose.pose.position.x
        goal.pose.point.y = self.current_drone_pose.pose.position.y
        goal.pose.point.z = coordinate.pose.position.z

        self.goal_pub.publish(goal)
        while self.reaching_delta_r(self.current_drone_pose, goal, self.delta_r) and not rospy.is_shutdown():
            if self.control_cmd == CmdManagerControlCmd.BREAK:
                self.stop_cmd()
                return False    # Если команда прервана возвращаем false

            elif self.control_cmd == CmdManagerControlCmd.PAUSE:
                self.stop_cmd()
                while self.control_cmd == CmdManagerControlCmd.PAUSE and not rospy.is_shutdown():
                    pass
                else:
                    self.goal_pub.publish(goal)     
        else:
            return True


    # FIXME: Полет по траектории
    def go_along_trajectory_cmd(self, coordinates: list) -> bool:
        if coordinates:
            for pose_point in coordinates:
                goal = Goal()
                goal.pose.point.x = pose_point.pose.position.x
                goal.pose.point.y = pose_point.pose.position.y
                goal.pose.point.z = self.working_hight #pose_point.pose.position.z
                self.goal_pub.publish(goal)     

                while self.reaching_delta_r(self.current_drone_pose, goal, self.delta_r) and not rospy.is_shutdown():  # and self.r_vel >= 0.03
                    if self.control_cmd == CmdManagerControlCmd.BREAK:
                        self.stop_cmd()
                        return False    # Если команда прервана возвращаем false

                    elif self.control_cmd == CmdManagerControlCmd.PAUSE:
                        self.stop_cmd()
                        while self.control_cmd == CmdManagerControlCmd.PAUSE and not rospy.is_shutdown():
                            pass
                        else:
                            self.goal_pub.publish(goal)     
            return True     # Если пройдена вся траектория возвращаем true
        else:
            print("go_along_trajectory_cmd ERROR - coordinates is empty")
            return False


    # FIXME: Команда полет в точку
    def go_to_point_cmd(self, coordinate: PoseStamped) -> bool:
        if self.use_global_planner:
            """
            req = PoseStamped
            coordinates = resp 
            """
            path = self.call_a_star(coordinate)    
            # print(path)
            return self.go_along_trajectory_cmd(path)
        else:
            goal = Goal()
            goal.pose.point = coordinate.pose.position
            yaw = euler_from_quaternion((coordinate.pose.orientation.x, coordinate.pose.orientation.y, coordinate.pose.orientation.z, coordinate.pose.orientation.w))[2]  # Вытаскиваем угол рыскания из кватерниона FIXME: Возможно придется кватернион взять в скобки
            goal.pose.course = yaw
            self.goal_pub.publish(goal)
            while self.reaching_delta_r(self.current_drone_pose, goal, self.delta_r) and not rospy.is_shutdown():
                if self.control_cmd == CmdManagerControlCmd.BREAK:
                    self.stop_cmd()
                    return False    # Если команда прервана возвращаем false

                elif self.control_cmd == CmdManagerControlCmd.PAUSE:
                    self.stop_cmd()
                    while self.control_cmd == CmdManagerControlCmd.PAUSE and not rospy.is_shutdown():
                        pass
                    else:
                        self.goal_pub.publish(goal)   
            else:
                return True


    # FIXME: Команда остановки
    def stop_cmd(self) -> bool:
        goal = Goal()
        goal.pose.point = self.current_drone_pose.pose.position
        yaw = euler_from_quaternion((self.current_drone_pose.pose.orientation.x, self.current_drone_pose.pose.orientation.y, self.current_drone_pose.pose.orientation.z, self.current_drone_pose.pose.orientation.w))[2]  # Вытаскиваем угол рыскания из кватерниона FIXME: Возможно придется кватернион взять в скобки
        goal.pose.course = yaw
        self.goal_pub.publish(goal)
        while self.reaching_delta_r(self.current_drone_pose, goal, self.delta_r) and not rospy.is_shutdown():
            if self.control_cmd == CmdManagerControlCmd.BREAK:
                self.stop_cmd()
                return False    # Если команда прервана возвращаем false

            elif self.control_cmd == CmdManagerControlCmd.PAUSE:
                self.stop_cmd()
                while self.control_cmd == CmdManagerControlCmd.PAUSE and not rospy.is_shutdown():
                    pass
                else:
                    self.goal_pub.publish(goal)     
        else:
            return True


    # FIXME: Команда возврат домой
    def return_cmd(self, return_path: list) -> bool:
        # return_path.extend(self.completed_path)
        # return_path.reverse()
        for goal_point in return_path:
            if self.control_cmd == CmdManagerControlCmd.BREAK:
                self.stop_cmd()
                break

            elif self.control_cmd == CmdManagerControlCmd.PAUSE:
                self.stop_cmd()
                while self.control_cmd == CmdManagerControlCmd.PAUSE and not rospy.is_shutdown():
                    pass

            elif self.control_cmd == CmdManagerControlCmd.START:
                self.goal_pub.publish(goal_point)     
                while self.reaching_delta_r(self.current_drone_pose, goal_point, self.delta_r) or not self.control_cmd == CmdManagerControlCmd.BREAK or not self.control_cmd == CmdManagerControlCmd.PAUSE and not rospy.is_shutdown():
                    pass
        return True


    # FIXME: Команда посадка 
    def land_cmd(self) -> None:
        rospy.wait_for_service('mavros/cmd/land')
        try:
            print("Выполнение посадки...")
            landingService = rospy.ServiceProxy('mavros/cmd/land', CommandTOL)
            info = landingService(0.0, 0.0, 0.0, 0.0, 0.0)  # TODO: сделать обратную связь по завершении посадки
            # print(info)
        except:
            pass


if __name__ == "__main__":
    rospy.init_node("task_manager_node")
    CommandManager()
    rospy.spin()

