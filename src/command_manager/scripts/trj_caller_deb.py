#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from drone_msgs.msg import GlobalTrajectory, Goal, DronePose, TaskCmd, CmdManagerControlCmd

class TrjDeb():
    def __init__(self):
        rospy.Subscriber("/goal_position", PoseStamped, self.goal_pose_cb)
        self.cmd_pub = rospy.Publisher("/task_manager/command", TaskCmd, queue_size=10)
        while not rospy.is_shutdown():
            pass

    def goal_pose_cb(self, msg: PoseStamped):
        cmd = TaskCmd()
        cmd.type = cmd.GO_TO_POINT
        cmd.coordinates.append(msg)
        print("========")
        print(cmd)
        self.cmd_pub.publish(cmd)
    


if __name__ == "__main__":
    rospy.init_node("trj_cmd_deb_node")
    TrjDeb()