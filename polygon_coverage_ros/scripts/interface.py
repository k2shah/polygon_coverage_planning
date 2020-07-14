#!/usr/bin/python3
"""
Copyright (c) 2019 Kunal Shah
                kshah.kunal@gmail.com
"""
# std lib imports
import sys
import time
import os
import os.path as osp

# std ROS imports
import rospy
import std_msgs.msg
from geometry_msgs.msg import PoseStamped, Vector3, PoseArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# special ROS service
from mav_planning_msgs.srv import PlannerService


class PlannerProxy(object):
    """This class implements a proxy for the communication between
        the user and the planner system"""

    def __init__(self):
        # Tell ROS this is a node
        rospy.init_node('plannerProxy', anonymous=True)

        # subs
        self.poseSub = rospy.Subscriber(
            "/waypoint_list",
            PoseArray,
            self.waypointCB)
        # service
        self.pathPlanSrv = rospy.ServiceProxy(
		"/coverage_planner/plan_path",
		PlannerService)
        self.rootDir = os.getcwd()
        self.outDir = "out"
        self.outName = "path.txt"
        if not os.path.exists(self.outDir):
            os.makedirs(self.outDir)
        self.outFile = osp.join(self.rootDir, self.outDir, self.outName)
        rospy.spin()
    def waypointCB(self, msg):
        # write the waypoint to a file
        print("got path. outputing to file")
        print(self.outFile)
        with open(self.outFile, "w+") as f:
            for pose in msg.poses:
                f.write(f"{pose.position.x}, {pose.position.y}, {pose.position.z}\n")

    def pathPlanSrv_call(self):
        # service inputs
        startPose = PoseStamped()
        startVel = Vector3()
        goalPose = PoseStamped()
        goalVel = Vector3()
        boundingBox = Vector3()

        # service call
        rsp = self.eLandService(
            startPose,
            startVel,
            goalPose,
            goalVel,
            boundingBox)
        return True


if __name__ == '__main__':
    plannerProxy = PlannerProxy()
