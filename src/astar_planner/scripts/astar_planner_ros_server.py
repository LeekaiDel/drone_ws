#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose, Point
from drone_msgs.msg import GlobalTrajectory
from astar_planner.srv import GetTrajectory, GetTrajectoryResponse
import math

class AStarPlanner:

    def __init__(self, oy, ox, resolution, rr):
        """
        Initialize grid map for a star planning
        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search
        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]
        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)

        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while 1:
            # print("HERE")
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]


            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        if len(open_set) == 0:
            return None, None
        else:
            ry, rx = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position
        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


class GlobalPlanner():
    def __init__(self):        
        self.trajectory_sended = True

        self.robot_pose_ = None
        self.goal_pose_  = PoseStamped()
        self.work_hight = 0.0

        self.grid_map_ = OccupancyGrid()
        self.old_map = OccupancyGrid()
        self.obstacle_map = list()

        self.global_path = list()
        self.old_path = list()
        self.old_goal_pose = PoseStamped()

        self.start_pose_grid = ()
        self.goal_pose_grid = ()

        # self.radius_of_robot = 0.8
        # self.filter_traj_threshold = 0.5
        # self.consider_unfound_area_flag = False

        self.radius_of_robot = rospy.get_param('~radius_of_robot', 0.8)
        self.filter_traj_threshold = rospy.get_param('~filter_traj_threshold', 0.5)
        self.consider_unfound_area_flag = rospy.get_param('~consider_unfound_area_flag', False)

        rospy.Subscriber("/map", OccupancyGrid, self.map_clb, queue_size=10)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.robot_pose_clb, queue_size=10)

        self.marker_path_pub = rospy.Publisher("/astar/viz/global_path", Marker, queue_size=10)
        self.marker_obs_pub = rospy.Publisher("/astar/viz/obstacles", MarkerArray, queue_size=10)   # TODO: сделать поинтклауд вместо маркеров, должно разгрузить
        
        self.service = rospy.Service('/astar/get_trajectory', GetTrajectory, self.get_trajectory_server) 

        rospy.spin()


    def get_trajectory_server(self, req):
        self.goal_pose_ = req.goal_position
        self.work_hight = req.work_hight.data
        # print("Get goal position :" + str(self.goal_pose_))
        self.trajectory_sended = False
        path = self.planner_loop()
        # print("Returned path :" + str(path))
        return GetTrajectoryResponse(path)


    def map_clb(self, msg: OccupancyGrid):
        """
        """
        self.grid_map_ = msg
        
    
    def robot_pose_clb(self, msg: PoseStamped):
        """
        """
        self.robot_pose_ = msg
    

    def goal_pose_clb(self, msg: PoseStamped):
        """
        """
        if self.old_goal_pose.pose.position != msg.pose.position:
            self.goal_pose_ = msg
            self.trajectory_sended = False


    def get_obstacle_map(self):
        """
        """
        self.obstacle_map.clear()
        for i in range(len(self.grid_map_.data)):
            if self.consider_unfound_area_flag:
                if self.grid_map_.data[i] > 50 or self.grid_map_.data[i] == -1: # Учитываем не также исследованную зону как препятствие 
                    obj_exist = i in self.obstacle_map
                    if obj_exist == False:
                        self.obstacle_map.append(i)
            else:
                if self.grid_map_.data[i] > 50:
                    obj_exist = i in self.obstacle_map
                    if obj_exist == False:
                        self.obstacle_map.append(i)


    def world_to_map(self, x:int, y:int):
        """
        Get map coordinates
        """
        mx = int((x - self.grid_map_.info.origin.position.x) / self.grid_map_.info.resolution)
        my = int((y - self.grid_map_.info.origin.position.y) / self.grid_map_.info.resolution)
        return [mx, my]
    

    def get_index(self, x:int, y:int):
        """
        Get index in map
        """
        return x + y * self.grid_map_.info.width
    

    def get_index_in_map_from_world_coords(self, x, y):
        """
        Returns index of point in world map to occupancy_grid
        """
        xm, ym = self.world_to_map(x, y)
        return self.get_index(xm, ym)
    

    def get_coords_from_grid_index(self, i):
        """
        Get coords using index in 1D occupancy grid 
        """
        y = divmod(i, self.grid_map_.info.width)[0]
        x = i - y * self.grid_map_.info.width
        
        # print(self.grid_map_.info)
        # print(y)
        
        x = x * self.grid_map_.info.resolution + self.grid_map_.info.origin.position.x + self.grid_map_.info.resolution / 2.0
        y = y * self.grid_map_.info.resolution + self.grid_map_.info.origin.position.y + self.grid_map_.info.resolution / 2.0
  
        return x, y
        

    def display_path(self, trajectory: list):
        # print("ok")
        marker_path = Marker()
        marker_path.id = 0
        marker_path.ns = "path"
        marker_path.type = Marker.LINE_STRIP
        marker_path.header.frame_id = "map"
        marker_path.action = Marker.ADD

        marker_path.color.a = 1.0
        marker_path.color.b = 0.0
        marker_path.color.g = 0.0
        marker_path.color.r = 1.0

        marker_path.scale.x = 0.05
        marker_path.scale.y = 0.05
        marker_path.scale.z = 0.0

        for i in range (len(trajectory)):
            waypoint = Point()
            # waypoint.header.stamp = rospy.get_time()
            waypoint.x = trajectory[i][0]
            waypoint.y = trajectory[i][1]
            waypoint.z = 0.1
            marker_path.points.append(waypoint)

        self.marker_path_pub.publish(marker_path)


    def display_obs(self, x:list, y:list):  # TODO: сделать поинтклаудом
        # print("OBS! " + str(len(x)))
        marker_arr = MarkerArray()
        for i in range(len(x)):
            # print("OBS")
            waypoint = Marker()
            waypoint.id = i
            waypoint.type = Marker.CUBE
            waypoint.header.frame_id = "map"
            waypoint.action = Marker.ADD

            # waypoint.header.stamp = self.get_clock().now().to_msg()
            waypoint.ns = "obst"
            waypoint.color.a = 1.0
            waypoint.color.b = 0.5
            waypoint.color.g = 0.0
            waypoint.color.r = 0.5

            waypoint.scale.x = self.grid_map_.info.resolution
            waypoint.scale.y = self.grid_map_.info.resolution
            waypoint.scale.z = self.grid_map_.info.resolution * 2

            waypoint.pose.position.x = x[i]
            waypoint.pose.position.y = y[i]
            waypoint.pose.position.z = 0.0


            marker_arr.markers.append(waypoint)
        self.marker_obs_pub.publish(marker_arr)
    

    def getAngleBetweenPoints(self, p1, p2):
        angle = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        angle = angle * 180 / math.pi

        if (angle < 0):
            angle = angle + 2 * 180

        return angle


    # Вычисление расстояния между точками
    def getDistanceBetweenPoints(self, p1, p2):
        distance = math.sqrt( math.pow(p2[0]-p1[0], 2) + math.pow(p2[1]-p1[1], 2) )
        return distance
    # Определение расстояния от точки до отрезка (при различных положениях точки относительно отрезка)
    # https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
    def getDistanceFromPointToLineSegment(self, p, p1, p2):
        A = p[0] - p1[0]
        B = p[1] - p1[1]
        C = p2[0] - p1[0]
        D = p2[1] - p1[1]

        dot = A * C + B * D
        len_sq = C * C + D * D
        param = -1
        if (len_sq != 0): # in case of 0 length line
            param = dot / len_sq

        xx = 0
        yy = 0

        # Рассматриваем только 3-ий вариант расположения точки относительно отрезка
        if (param > 0) and (param < 1): # 3 случай
            xx = p1[0] + param * C
            yy = p1[1] + param * D

        dx = p[0] - xx
        dy = p[1] - yy

        distance = math.sqrt(dx * dx + dy * dy)

        if (param < 0) or (param > 1):           # 1 случай
            distance = -1

        return distance
    

    # Фильтр ступенек заданного размера
    def filter_trajectory_by_saw(self, trajectory, saw_size):
        start_pos_index = -1
        while(True):
            angle_joint_1 = -1
            angle_joint_2 = -1
            distance_joint_1 = -1
            distance_joint_2 = -1

            start_pos_index = start_pos_index + 1

            if (start_pos_index + 1 <= len(trajectory) - 1):
                angle_joint_1 = self.getAngleBetweenPoints(trajectory[start_pos_index],trajectory[start_pos_index + 1])
                distance_joint_1 = self.getDistanceBetweenPoints(trajectory[start_pos_index],trajectory[start_pos_index + 1])

            if (start_pos_index + 2 <= len(trajectory) - 1):
                angle_joint_2 = self.getAngleBetweenPoints(trajectory[start_pos_index + 1], trajectory[start_pos_index + 2])
                distance_joint_2 = self.getDistanceBetweenPoints(trajectory[start_pos_index + 1], trajectory[start_pos_index + 2])

            # Пила под прямым углом
            if (math.fabs(angle_joint_1 - angle_joint_2) == 90) and (distance_joint_1 == distance_joint_2): # Пила обнаружена
                if (distance_joint_1 <= saw_size):
                    del trajectory[start_pos_index + 1]
                    start_pos_index = 0

            # Пила под косым углом
            if (start_pos_index + 1 <= len(trajectory) - 1) and (start_pos_index + 2 <= len(trajectory) - 1):
                if (distance_joint_1 <= saw_size) and (distance_joint_2 <= saw_size) and (angle_joint_1 != angle_joint_2):
                    saw_height = self.getDistanceFromPointToLineSegment(trajectory[start_pos_index + 1], trajectory[start_pos_index],trajectory[start_pos_index + 2])
                    if (saw_height != -1):
                        if (saw_height <= saw_size * 0.5):
                            del trajectory[start_pos_index + 1]
                            start_pos_index = 0

            if (start_pos_index >= len(trajectory) - 1):
                break

        return trajectory


    # Фильтр по углу между сочленениями
    def filter_trajectory_by_angle(self, trajectory, angle, distance):
        start_pos_index = 0

        while(True):
            angle_joint_1 = -1
            angle_joint_2 = -1
            distance_joint_1 = -1
            distance_joint_2 = -1

            if (start_pos_index + 1 <= len(trajectory) - 1):
                angle_joint_1 = self.getAngleBetweenPoints(trajectory[start_pos_index], trajectory[start_pos_index + 1])
                distance_joint_1 = self.getDistanceBetweenPoints(trajectory[start_pos_index], trajectory[start_pos_index + 1])

            if (start_pos_index + 2 <= len(trajectory) - 1):
                angle_joint_2 = self.getAngleBetweenPoints(trajectory[start_pos_index + 1], trajectory[start_pos_index + 2])
                distance_joint_2 = self.getDistanceBetweenPoints(trajectory[start_pos_index + 1], trajectory[start_pos_index + 2])

            if ((angle_joint_1 != -1) and (angle_joint_2 != -1)):
                if (math.fabs(angle_joint_1 - angle_joint_2) == 0):
                    del trajectory[start_pos_index + 1]
                    start_pos_index = 0
                elif (math.fabs(angle_joint_1 - angle_joint_2) <= angle) and (distance_joint_1 + distance_joint_2 <= distance):
                    del trajectory[start_pos_index + 1]
                    start_pos_index = 0
                elif (math.fabs(angle_joint_1 - angle_joint_2) <= 15) and (distance_joint_1 <= 5 or distance_joint_2 <= 5):
                    del trajectory[start_pos_index + 1]
                    start_pos_index = 0
                elif angle_joint_1 == 0 or angle_joint_2 == 0:
                    if (angle_joint_1 == 0):
                        angle_joint_1 = 360

                    if (angle_joint_2 == 0):
                        angle_joint_2 = 360

                    if (math.fabs(angle_joint_1 - angle_joint_2) <= angle) and (distance_joint_1 + distance_joint_2 <= distance):
                        del trajectory[start_pos_index + 1]
                        start_pos_index = 0
                    else:
                        start_pos_index = start_pos_index + 1
                else:
                    start_pos_index = start_pos_index + 1
            else:
                break

        return trajectory


    def planner_loop(self):
        if (self.robot_pose_ == None or self.grid_map_ == None or self.trajectory_sended == True):
            return

        obs_x = []
        obs_y = []

        self.get_obstacle_map()

        for i in self.obstacle_map:
            x, y = self.get_coords_from_grid_index(i)
            obs_x.append(x)
            obs_y.append(y)
            # print(f"OBSTACLE {i} {[x, y]}")
        self.display_obs(obs_x, obs_y)

        a_star = AStarPlanner(obs_x, obs_y, self.grid_map_.info.resolution,  self.radius_of_robot)

        # self.start_pose_grid = [0, 0]
        rx, ry = a_star.planning(self.robot_pose_.pose.position.y, self.robot_pose_.pose.position.x, self.goal_pose_.pose.position.y, self.goal_pose_.pose.position.x)
        
        if rx is None and ry is None:
            self.trajectory_sended = True
            print("The trajectory is not found!")
            return

        rx.reverse()
        ry.reverse()

        trajectory = list()
        for i in range(len(rx)):
            waypoint = list()
            waypoint.append(rx[i])
            waypoint.append(ry[i])
            trajectory.append(waypoint)

        # trajectory = self.filter_trajectory_by_saw(trajectory, self.filter_traj_threshold)
        trajectory = self.filter_trajectory_by_angle(trajectory, 0.1, 0.5)
        print("Trajectory output: " + str(len(trajectory)))

        self.display_path(trajectory)

        output_path = list()
        for i in range(len(trajectory)):
            waypoint = PoseStamped()
            waypoint.pose.position.x = trajectory[i][0]
            waypoint.pose.position.y = trajectory[i][1]
            waypoint.pose.position.z = self.work_hight
            output_path.append(waypoint)

        self.trajectory_sended = True
        self.old_goal_pose = self.goal_pose_
        return output_path


if __name__ == "__main__":

    rospy.init_node("astar_global_planner")
    GlobalPlanner()
    rospy.spin()
    rospy.destroy_node()
    rospy.shutdown()
