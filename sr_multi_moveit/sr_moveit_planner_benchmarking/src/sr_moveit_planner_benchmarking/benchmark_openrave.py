#!/usr/bin/env python

import rospy
import time
import os
from openravepy import *

class OpenraveUtils(object):
    def __init__(self, env):
        self.env = env

    def load_robot(self, urdf_path, srdf_path):
        # Get robot from urdf and srdf
        plugin = RaveCreateModule(self.env, "urdf")
        robot_name = plugin.SendCommand("LoadURI " + urdf_path + " " + srdf_path)
        return robot_name

    def set_viewer(self, on):
        if on:
            self.env.SetViewer('qtcoin')

    def load_target(self, target_path, target_name):
        target = self.env.ReadKinBodyURI(target_path)
        target.SetName(target_name)
        self.env.Add(target)
        return target

class OpenRAVEPlanner(object):
    def __init__(self, viewer=True):
        self.env = Environment()
        or_utils = OpenraveUtils(self.env)
        or_utils.set_viewer(viewer)
        # RaveSetDebugLevel(DebugLevel.Debug)

        robot_files_path = rospy.get_param('~robot_files_path', "/tmp/")
        robot_file_name = rospy.get_param('~robot_file_name', "generated_robot")

        self.env.Load("/tmp/model.xml")
        self.robot = self.env.GetRobots()[0]
        print "robot", self.robot.GetName()
        self.manip = self.robot.GetActiveManipulator()
        print "manip:", self.manip.GetName()

        print self.robot.GetLinks()[2].GetTransform()
        print self.robot.GetLinks()[2].GetName()

        ikmodel = databases.inversekinematics.InverseKinematicsModel(self.robot, iktype=IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            print "Autogenerating ikmodel"
            ikmodel.autogenerate()

        self.manipprob = interfaces.BaseManipulation(self.robot)  # create the interface for basic manipulation programs
        raw_input("Press a key to start the test")

        self.test([0.11, 0.00, -0.93, -2.22, -1.71, -1.68], [1.55, -1.15, 2.01, 2.39, -1.55, -1.58])
        self.test([2.012, -0.639, 1.827, 2.019, -1.390, -3.056], [2.675, -0.852, 1.202, 2.822, -1.724, -1.615])
        self.test([-1.489, -2.032, 2.080, -2.777, -1.522, 3.139], [-0.387, -1.445, 1.739, -3.002, -2.2003, -2.752])

        raw_input("key to continue")

    def move_to_joint_value(self, joints):
        print "Testing in joint space"
        with self.env:
            jointnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            self.robot.SetActiveDOFs([self.robot.GetJoint(name).GetDOFIndex() for name in jointnames])
            try:
                # self.manipprob.MoveActiveJoints(goal=joints,maxiter=5000,steplength=0.15,maxtries=5)
                plan = self.manipprob.MoveActiveJoints(goal=joints, maxiter=5000, steplength=0.01, maxtries=2,
                                                       execute=False, outputtrajobj=True)
                plan_quality = self.evaluate_plan(plan)
                print "plan_quality", plan_quality

            except planning_error:
                rospy.logerr("Planning error")
                return
        self.robot.WaitForController(0)  # wait

    def evaluate_plan(self, plan):
        num_of_joints = len(self.manip.GetArmIndices())
        weights = numpy.array(sorted(range(1, num_of_joints + 1), reverse=True))
        plan_array = numpy.empty(shape=(plan.GetNumWaypoints(), num_of_joints))

        for i in range(plan.GetNumWaypoints()):
            plan_array[i] = plan.GetWaypoint(i)[0:6]

        deltas = abs(numpy.diff(plan_array, axis=0))
        sum_deltas = numpy.sum(deltas, axis=0)
        sum_deltas_weighted = sum_deltas * weights
        plan_quality = numpy.sum(sum_deltas_weighted)
        return plan_quality

    def set_joint_values(self, joints):
        jointnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.robot.SetActiveDOFs([self.robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        self.robot.SetActiveDOFValues(joints)
        rospy.sleep(5.0)

    def move_to_pose(self, position, quaternion):
        print "Testing in cartesian space"
        matrix = self.getMatrixFromPose(position,quaternion)
        Tgoal = numpy.array(matrix)
        plan = self.manipprob.MoveToHandPosition(matrices=[Tgoal], seedik=10, execute=False, outputtrajobj=True)
        plan_quality = self.evaluate_plan(plan)
        print "plan_quality", plan_quality

        self.robot.WaitForController(0)  # wait

    def getPoseFromMatrix(self, matrix):
        # Openrave function returns the quaternion first than translation, so order should be inverted
        pose_q_t = poseFromMatrix(matrix)
        pose_t_q = [0.0] * 7
        pose_t_q[0:3] = pose_q_t[4:7]
        pose_t_q[3] = pose_q_t[1]
        pose_t_q[4] = pose_q_t[2]
        pose_t_q[5] = pose_q_t[3]
        pose_t_q[6] = pose_q_t[0]
        return pose_t_q

    def getMatrixFromPose(self, position, quaternion):
        pose_or_format = [0.0] * 7
        pose_or_format[0] = quaternion[3]
        pose_or_format[1] = quaternion[0]
        pose_or_format[2] = quaternion[1]
        pose_or_format[3] = quaternion[2]
        pose_or_format[4:7] = position
        return matrixFromPose(pose_or_format)

    def test(self, start_joints, goal_joints):
        self.set_joint_values(start_joints)
        print "start xyz", self.getPoseFromMatrix(self.manip.GetTransform())
        self.move_to_joint_value(goal_joints)

        self.set_joint_values(goal_joints)
        pose = self.getPoseFromMatrix(self.manip.GetTransform())
        print "goal xyz:", pose
        self.set_joint_values(start_joints)
        self.move_to_pose(pose[0:3],pose[3:7])


if __name__ == '__main__':
    rospy.init_node("openrave_planner_benchmark")

    OpenRAVEPlanner()
