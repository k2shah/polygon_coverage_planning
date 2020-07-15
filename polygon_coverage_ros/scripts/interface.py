#!/usr/bin/python3
"""
Copyright (c) 2019 Kunal Shah
                kshah.kunal@gmail.com
"""
# std lib imports
import os
import os.path as osp
import numpy as np
import matplotlib.pyplot as plt

# std ROS imports
import rospy
import std_msgs.msg
from geometry_msgs.msg import PoseStamped, Vector3, PoseArray
# special ROS service
from mav_planning_msgs.srv import PlannerService


class PlannerProxy(object):
    """This class implements a proxy for the communication between
        the user and the planner system"""

    def __init__(self):
        # Tell ROS this is a node
        rospy.init_node('plannerProxy', anonymous=True)

        # prep output location
        rootDir = osp.dirname(osp.abspath(__file__))
        outDir = osp.join(rootDir, "out")
        outName = "path_1"
        if not osp.exists(outDir):
            os.makedirs(outDir)
        self.outFile = osp.join(outDir, outName)

        # subs
        self.poseSub = rospy.Subscriber(
            "/waypoint_list",
            PoseArray,
            self.waypointCB)
        # service
        self.pathPlanSrv = rospy.ServiceProxy(
            "/coverage_planner/plan_path",
            PlannerService)

        self.run()

    def waypointCB(self, msg):
        # write the waypoint to a file
        self.writePath(msg.poses)
        self.plotPath(msg.poses)

    def writePath(self, path):
        # path: array of Poses
        outFile = self.outFile + ".txt"
        print("writing to file: " + outFile)
        with open(outFile, "w+") as f:
            for pose in path:
                f.write(f"{pose.position.x}," +
                        f"{pose.position.y}," +
                        f"{pose.position.z}\n")

    def plotPath(self, path):
        outFile = self.outFile + ".png"
        path = np.array([[pose.position.x, pose.position.y] for pose in path])
        plt.plot(path[:, 0], path[:, 1])
        plt.savefig(outFile, dpi=200, bbox_inches='tight')

    def pathPlanSrv_call(self):
        # service inputs
        startPose = PoseStamped()
        startVel = Vector3()
        goalPose = PoseStamped()
        goalVel = Vector3()
        boundingBox = Vector3()

        startPose.pose.position.x = 78639.0
        startPose.pose.position.y = 1472530.0

        #start = goal
        goalPose.pose= startPose.pose

        # service call
        rsp = self.pathPlanSrv(
            startPose,
            startVel,
            goalPose,
            goalVel,
            boundingBox)
        print(rsp)
        return True

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            print("c to start, x to exit")
            usrCmd = input("Enter a command: ")
            if usrCmd == 'c':
                self.pathPlanSrv_call()
            elif usrCmd == 'x':
                break
            else:
                print('Not a valid command')
            rate.sleep()


if __name__ == '__main__':
    plannerProxy = PlannerProxy()
