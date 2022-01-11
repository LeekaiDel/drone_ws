#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Pose, Point
from drone_msgs.msg import GlobalTrajectory
import math

class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
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

        rx, ry = self.calc_final_path(goal_node, closed_set)

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
        self.robot_id = 1
        if self.robot_id == None : self.robot_id = 0

        self.trajectory_sended = True

        self.robot_pose_ = None
        self.goal_pose_  = PoseStamped()
        self.trajectory = GlobalTrajectory()

        self.grid_map_ = None
        self.old_map = OccupancyGrid()
        self.obstacle_map = list()

        self.global_path = list()
        self.old_path = list()
        self.old_goal_pose = PoseStamped()

        self.start_pose_grid = ()
        self.goal_pose_grid = ()

        robot_topic_prefix = ""

        rospy.Subscriber("/map", OccupancyGrid, self.map_clb, queue_size=10)
        rospy.Subscriber(robot_topic_prefix + "/mavros/local_position/pose", PoseStamped, self.robot_pose_clb, queue_size=10)
        rospy.Subscriber(robot_topic_prefix + "/end_goal_pose", PoseStamped, self.goal_pose_clb, queue_size=10)

        self.send_trajectory_pub = rospy.Publisher(robot_topic_prefix + "/trajectory", GlobalTrajectory,  queue_size=10)
        # self.visualize_global_path = rospy.Publisher("/global_path", MarkerArray, queue_size=10)
        self.marker_path_pub = rospy.Publisher("/global_path", Marker, queue_size=10)
        self.marker_obs_pub = rospy.Publisher("/obstacles", MarkerArray, queue_size=10)
        
        while not rospy.is_shutdown():
            self.planner_loop()

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
        for i in range(len(self.grid_map_.data)):
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
        x = divmod(i, self.grid_map_.info.height)
        y = divmod(i, self.grid_map_.info.width)

        x = x[0] * self.grid_map_.info.resolution + self.grid_map_.info.origin.position.x + self.grid_map_.info.resolution
        y = y[1] * self.grid_map_.info.resolution + self.grid_map_.info.origin.position.y + self.grid_map_.info.resolution
  
        return x, y
        

    def planner_loop(self):
        
        if (self.robot_pose_ == None or self.grid_map_ == None or self.trajectory_sended == True):
            return

        obs_x = []
        obs_y = []

        self.get_obstacle_map()
        for i in self.obstacle_map:
            x,y = self.get_coords_from_grid_index(i)
            obs_x.append(x)
            obs_y.append(y)
            print(f"OBSTACLE {i} {[x, y]}")
        self.display_obs(obs_y, obs_x)

        a_star = AStarPlanner(obs_x, obs_y, self.grid_map_.info.resolution, 0.7)

        # self.start_pose_grid = [0, 0]
        rx, ry = a_star.planning(self.robot_pose_.pose.position.y, self.robot_pose_.pose.position.x, self.goal_pose_.pose.position.y, self.goal_pose_.pose.position.x)
        self.display_path(ry, rx)
        
        self.infill_trajectory(ry, rx)
        self.trajectory_sended = True
        self.old_goal_pose = self.goal_pose_
        

    def display_path(self, x: list, y: list):
        print("ok")
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

        marker_path.scale.x = 0.1
        marker_path.scale.y = 0.1
        marker_path.scale.z = 0.0

        for i in range (len(x)):
            waypoint = Point()
            # waypoint.header.stamp = rospy.get_time()
            waypoint.x = x[i]
            waypoint.y = y[i]
            waypoint.z = 0.1
            marker_path.points.append(waypoint)

        self.marker_path_pub.publish(marker_path)
        # self.trajectory = GlobalTrajectory()

    def display_obs(self, x:list, y:list):
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
            waypoint.scale.z = self.grid_map_.info.resolution * 10

            waypoint.pose.position.x = x[i]
            waypoint.pose.position.y = y[i]
            waypoint.pose.position.z = 0.0


            marker_arr.markers.append(waypoint)
        self.marker_obs_pub.publish(marker_arr)
    
    def infill_trajectory(self, x, y):
        x.reverse()
        y.reverse()

        self.trajectory.mode = GlobalTrajectory.WITHOUT_PLANNERS
        for i in range(len(x)):
            waypoint = PoseStamped()
            waypoint.pose.position.x = x[i]
            waypoint.pose.position.y = y[i]

            self.trajectory.waypoints.append(waypoint)
        self.send_trajectory_pub.publish(self.trajectory)

if __name__ == "__main__":

    rospy.init_node("astar_global_planner")
    GlobalPlanner()
    rospy.spin()
    rospy.destroy_node()
    rospy.shutdown()
