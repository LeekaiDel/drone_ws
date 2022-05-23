#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseStamped, Pose, Point
from drone_msgs.msg import GlobalTrajectory, Goal, DronePose, TaskCmd, TaskManagerControlCmd
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
import math
import threading
import sys
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class TaskManager():
    def __init__(self):
        # params:
        self.delta_r = 0.1              # Дельта окрестность достижения точки

        # variables:
        self.working_hight = 0.5        # Рабочая высота
        
        self.drone_state = ExtendedState()  
        self.current_drone_pose = PoseStamped()

        self.completed_path = list()    # Пройденный маршрут
        self.current_task =  None       # Текущее задание

        rospy.Subscriber("/mavros/extended_state", ExtendedState, self.drone_state_cb, queue_size=10)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.drone_pose_cb, queue_size=10)
        rospy.Subscriber("/task_manager/task", TaskCmd, self.task_cb, queue_size=10)
        
        self.task_status_pub = rospy.Publisher("/task_manager/task_status", Goal, queue_size=10)
        self.goal_pub = rospy.Publisher("/goal_pose", Goal, queue_size=10)
        
        # thread_main = threading.Thread(target=self.main, daemon=True)
        # thread_main.start()
        

        # Функция проверяет наличие нового задания и передает траекторию на регулятор
        while True:
            pass


    # Получаем данные о позиции дрона
    def drone_pose_cb(self, msg: PoseStamped) -> None:
        self.current_drone_pose = msg


    # Получаем самую свежую траекторию
    def task_cb(self, msg: TaskCmd) -> None:
        self.current_task = msg


    # Функция проеряет достигнута ли целевая точка по условию дельта-окрестности
    def _reaching_delta_r(self, current_pose: PoseStamped, goal_pose: Goal, delta_r) -> bool:
        return round(math.sqrt((goal_pose.pose.point.x - current_pose.pose.position.x)**2 + (goal_pose.pose.point.y - current_pose.pose.position.y)**2), 1) >= delta_r


    # FIXME: Команда взлет
    def takeoff_cmd(self, coordinate: PoseStamped) -> None:
        goal = Goal()
        goal.pose.point.x = self.current_drone_pose.pose.position.x
        goal.pose.point.y = self.current_drone_pose.pose.position.y
        goal.pose.point.z = coordinate.pose.position.z
        self.goal_pub.publish(goal)


    # FIXME: Команда полет в точку
    def go_to_point_cmd(self, coordinate: PoseStamped) -> None:
        goal = Goal()
        goal.pose.point = coordinate.pose.position
        yaw = euler_from_quaternion(coordinate.pose.orientation)[2]  # Вытаскиваем угол рыскания из кватерниона FIXME: Возможно придется кватернион взять в скобки
        goal.pose.course = yaw
        self.goal_pub.publish(goal)


    # TODO: Полет по траектории
    def go_along_trajectory_cmd(self, coordinates: list) -> None:
        for pose_point in coordinates:
            goal = Goal()
            goal.pose.point.x = pose_point.pose.position.x
            goal.pose.point.y = pose_point.pose.position.y
            goal.pose.point.z = pose_point.pose.position.z      # FIXME: решить какая будет высота

            self.goal_pub.publish(goal)     
            while self._reaching_delta_r(self.current_drone_pose, goal, self.delta_r):
                pass


    # FIXME: Команда остановки
    def stop_cmd(self) -> None:
        goal = Goal()
        goal.pose.point = self.curent_drone_pose.pose.position
        self.goal_pub.publish(goal)


    # FIXME: Команда возврат домой
    def return_cmd(self, return_path: list) -> bool:
        # return_path.extend(self.completed_path)
        # return_path.reverse()
        for goal_point in return_path:
            self.goal_pub.publish(goal_point)
            while self._reaching_delta_r(self.current_drone_pose, goal_point, self.delta_r):
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

