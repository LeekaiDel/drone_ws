#!/usr/bin/env python3
from cmd import Cmd
import rospy
from geometry_msgs.msg import PoseStamped
from drone_msgs.msg import GlobalTrajectory, Goal, DronePose, TaskCmd, CmdManagerControlCmd

class TrjDeb():
    def __init__(self):
        rospy.Subscriber("/goal_position", PoseStamped, self.goal_pose_cb)
        self.cmd_pub = rospy.Publisher("/command_manager/command", TaskCmd, queue_size=10)
        self.control_cmd_pub = rospy.Publisher("/command_manager/control_cmd", CmdManagerControlCmd, queue_size=10)
        cmd = CmdManagerControlCmd()
        while not rospy.is_shutdown():
            command = input("________\n0 - Stop\n1 - Start\n2 - Pause\ne - Exit\n")
            if command == '0':
                cmd.control_cmd = CmdManagerControlCmd.BREAK
            elif command == '1':
                cmd.control_cmd = CmdManagerControlCmd.START
            elif command == '2':
                cmd.control_cmd = CmdManagerControlCmd.PAUSE
            elif command == 'e':
                break
            else:
                continue
            self.control_cmd_pub.publish(cmd)

    def goal_pose_cb(self, msg: PoseStamped):
        cmd = TaskCmd()
        cmd.type = cmd.GO_TO_POINT
        cmd.coordinates.append(msg)
        # print("========")
        # print(cmd)
        self.cmd_pub.publish(cmd)
    


if __name__ == "__main__":
    rospy.init_node("trj_cmd_deb_node")
    TrjDeb()