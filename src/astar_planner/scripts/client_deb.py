#!/usr/bin/env python3

import rospy
from astar_planner.srv import GetTrajectory
from geometry_msgs.msg import PoseStamped

class ClientDeb:
    def __init__(self):
        rospy.init_node("client_debug_astar_node")
        rospy.Subscriber("/goal_position", PoseStamped, self.pose_cb)
        rospy.spin()


    def get_response(self, goal_position):
        rospy.wait_for_service('/astar/get_trajectory')
        try:
            get_path_service = rospy.ServiceProxy('/astar/get_trajectory', GetTrajectory)
            resp = get_path_service(goal_position)
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            
    
    def pose_cb(self, msg):
        path = self.get_response(msg)
        print("Returned :" + str(path))
        

if __name__ == "__main__":
    ClientDeb()