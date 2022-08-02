#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Pose, Point, TwistStamped
from drone_msgs.msg import GlobalTrajectory, Goal, DronePose, TaskCmd, TaskManagerControlCmd
from mavros_msgs.msg import State, ExtendedState 
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from astar_planner.srv import GetTrajectory
import math
import threading
import sys
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class TaskManager():
    def __init__(self):
        # params:
        self.delta_r = 0.1                                  # Дельта окрестность достижения точки
        self.use_global_planner = True

        # variables:
        self.working_hight = 0.5                            # Рабочая высота
        
        self.drone_state = ExtendedState()  
        self.current_drone_pose = PoseStamped()             # Текущая позиция дрона
        self.current_velocity = TwistStamped()              # Текущая скорость дрона

        self.completed_path = list()                        # Пройденный маршрут
        self.current_task =  TaskCmd()                      # Текущее задание
        self.control_cmd = TaskManagerControlCmd.FINISH     # Команда управления менеджером

        # flags:
        self.get_new_cmd = False
        self.stop_command = False

        # ros initialisations:
        # rospy.Subscriber("/mavros/extended_state", ExtendedState, self.drone_state_cb, queue_size=10)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.drone_pose_cb, queue_size=10)
        rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self.local_vel_cb, queue_size=10)
        rospy.Subscriber("/task_manager/task", TaskCmd, self.task_cb, queue_size=10)
        rospy.Subscriber("/task_manager/control_cmd", TaskManagerControlCmd, self.control_cmd_cb, queue_size=10)

        self.task_status_pub = rospy.Publisher("/task_manager/task_status", Goal, queue_size=10)
        self.goal_pub = rospy.Publisher("/goal_pose", Goal, queue_size=10)
        
        # thread_main = threading.Thread(target=self.main, daemon=True)
        # thread_main.start()


        # Функция проверяет наличие нового задания и передает траекторию на регулятор
        while not rospy.is_shutdown():
            if self.get_new_cmd == True:
                # Взлет
                if self.current_task.type == TaskCmd.TAKEOFF:
                    if self.current_task.coordinates:
                        print("Команда взлет: " + str(self.takeoff_cmd(coordinate = self.current_task.coordinates[0])))


                # Полет в точку
                elif self.current_task.type == TaskCmd.GO_TO_POINT:
                    self.go_to_point_cmd(self.current_task.coordinates)


                # Полет по траектории
                elif self.current_task.type == TaskCmd.GO_ALONG_TRAJECTORY:
                    self.go_along_trajectory_cmd(coordinates=self.current_task.coordinates)


                # Стоп
                elif self.current_task.type == TaskCmd.STOP:
                    print("Команда стоп: " + str(self.stop_cmd()))


                # Возврат
                elif self.current_task.type == TaskCmd.RETURN:
                    pass


                # Посадка
                elif self.current_task.type == TaskCmd.LAND:
                    self.land_cmd()


                else:
                    pass

                self.current_task = TaskCmd()
                self.get_new_cmd = False

    # Получаем данные о позиции дрона
    def drone_pose_cb(self, msg: PoseStamped) -> None:
        self.current_drone_pose = msg


    # Получаем самую свежую траекторию
    def task_cb(self, msg: TaskCmd) -> None:
        print("Get new task")
        self.current_task = msg
        self.get_new_cmd = True
    

    # Получаем команды управления менеджером
    def control_cmd_cb(self, msg: TaskManagerControlCmd):
        self.control_cmd = msg.control_cmd


    # Получаем локальную скорость дрона
    def local_vel_cb(self, msg: TwistStamped):
        self.current_velocity = msg


    # Функция проеряет достигнута ли целевая точка по условию дельта-окрестности
    def _reaching_delta_r(self, current_pose: PoseStamped, goal_pose: Goal, delta_r) -> bool:
        return round(math.sqrt((goal_pose.pose.point.x - current_pose.pose.position.x)**2 + (goal_pose.pose.point.y - current_pose.pose.position.y)**2) + (goal_pose.pose.point.z - current_pose.pose.position.z)**2, 1) >= delta_r


    # Функция вызова запроса к А*
    def call_a_star(self, goal_position): 
        rospy.wait_for_service('/astar/get_trajectory')
        try:
            get_path_service = rospy.ServiceProxy('/astar/get_trajectory', GetTrajectory)
            resp = get_path_service(goal_position)
            return resp
        except rospy.ServiceException as e:
            print("Service AStar call failed: %s"%e)
            


    # FIXME: Команда взлет
    def takeoff_cmd(self, coordinate: PoseStamped) -> bool:
        goal = Goal()
        goal.pose.point.x = self.current_drone_pose.pose.position.x
        goal.pose.point.y = self.current_drone_pose.pose.position.y
        goal.pose.point.z = coordinate.pose.position.z
        self.goal_pub.publish(goal)
        while self._reaching_delta_r(self.current_drone_pose, goal, self.delta_r):
            if self.control_cmd == TaskManagerControlCmd.BRAKE:
                self.stop_cmd()
                return False    # Если команда прервана возвращаем false

            elif self.control_cmd == TaskManagerControlCmd.PAUSE:
                self.stop_cmd()
                while self.control_cmd == TaskManagerControlCmd.PAUSE:
                    pass
                else:
                    self.goal_pub.publish(goal)     
        else:
            return True


    # FIXME: Полет по траектории
    def go_along_trajectory_cmd(self, coordinates: list) -> bool:
        for pose_point in coordinates:
            goal = Goal()
            goal.pose.point.x = pose_point.pose.position.x
            goal.pose.point.y = pose_point.pose.position.y
            goal.pose.point.z = pose_point.pose.position.z
            
            self.goal_pub.publish(goal)     
            while self._reaching_delta_r(self.current_drone_pose, goal, self.delta_r):
                if self.control_cmd == TaskManagerControlCmd.BRAKE:
                    self.stop_cmd()
                    return False    # Если команда прервана возвращаем false

                elif self.control_cmd == TaskManagerControlCmd.PAUSE:
                    self.stop_cmd()
                    while self.control_cmd == TaskManagerControlCmd.PAUSE:
                        pass
                    else:
                        self.goal_pub.publish(goal)     
        return True     # Если пройдена вся траектория возвращаем true


    # FIXME: Команда полет в точку
    def go_to_point_cmd(self, coordinate: PoseStamped) -> bool:
        if self.use_global_planner:
            """
            req = PoseStamped
            coordinates = resp 
            """
            path = self.call_a_star(coordinate)    
            if self.go_along_trajectory_cmd(path):
                return True

        else:
            goal = Goal()
            goal.pose.point = coordinate.pose.position
            yaw = euler_from_quaternion((coordinate.pose.orientation.x, coordinate.pose.orientation.y, coordinate.pose.orientation.z, coordinate.pose.orientation.w))[2]  # Вытаскиваем угол рыскания из кватерниона FIXME: Возможно придется кватернион взять в скобки
            goal.pose.course = yaw
            self.goal_pub.publish(goal)
            while self._reaching_delta_r(self.current_drone_pose, goal, self.delta_r):
                if self.control_cmd == TaskManagerControlCmd.BRAKE:
                    self.stop_cmd()
                    return False    # Если команда прервана возвращаем false

                elif self.control_cmd == TaskManagerControlCmd.PAUSE:
                    self.stop_cmd()
                    while self.control_cmd == TaskManagerControlCmd.PAUSE:
                        pass
                    else:
                        self.goal_pub.publish(goal)   
            else:
                return True


    # FIXME: Команда остановки
    def stop_cmd(self) -> None:
        goal = Goal()
        goal.pose.point = self.current_drone_pose.pose.position
        # print()
        yaw = euler_from_quaternion((self.current_drone_pose.pose.orientation.x, self.current_drone_pose.pose.orientation.y, self.current_drone_pose.pose.orientation.z, self.current_drone_pose.pose.orientation.w))[2]  # Вытаскиваем угол рыскания из кватерниона FIXME: Возможно придется кватернион взять в скобки
        goal.pose.course = yaw
        self.goal_pub.publish(goal)
        while self._reaching_delta_r(self.current_drone_pose, goal, self.delta_r):
            if self.control_cmd == TaskManagerControlCmd.BRAKE:
                self.stop_cmd()
                return False    # Если команда прервана возвращаем false

            elif self.control_cmd == TaskManagerControlCmd.PAUSE:
                self.stop_cmd()
                while self.control_cmd == TaskManagerControlCmd.PAUSE:
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
            if self.control_cmd == TaskManagerControlCmd.BRAKE:
                self.stop_cmd()
                break

            elif self.control_cmd == TaskManagerControlCmd.PAUSE:
                self.stop_cmd()
                while self.control_cmd == TaskManagerControlCmd.PAUSE:
                    pass

            elif self.control_cmd == TaskManagerControlCmd.START:
                self.goal_pub.publish(goal_point)     
                while self._reaching_delta_r(self.current_drone_pose, goal_point, self.delta_r) or not self.control_cmd == TaskManagerControlCmd.BRAKE or not self.control_cmd == TaskManagerControlCmd.PAUSE:
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
    TaskManager()
    rospy.spin()

