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
        robot = self.env.GetRobots()[0]
        print "robot", robot.GetName()
        manip = robot.GetActiveManipulator()
        print "manip:", manip.GetName()

        ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            print "Autogenerating ikmodel"
            ikmodel.autogenerate()

        print "Loading robot"
        self.env.Load("data/mug1.kinbody.xml")

        mug = self.env.GetKinBody('mug')
        mug_pose = numpy.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0.05], [0, 0, 0, 1]])

        mug.SetTransform(mug_pose)

        manipprob = interfaces.BaseManipulation(robot)  # create the interface for basic manipulation programs

        raw_input("Press a key to start:")

        Tgoal = numpy.array([[0.37, 0.92, 0, 0.4], [0, 0, 1, -0.23], [0.93, -0.37, 0, 0.12], [0, 0, 0, 1]])
        res = manipprob.MoveToHandPosition(matrices=[Tgoal], seedik=10)  # call motion planner with goal joint angles
        robot.WaitForController(0)  # wait

        manipprob = interfaces.BaseManipulation(robot)  # create the interface for basic manipulation programs
        Tgoal = numpy.array([[0.51, 0.86, 0, -0.4], [0, 0, 1, -0.23], [0.86, -0.51, 0, 0.12], [0, 0, 0, 1]])
        res = manipprob.MoveToHandPosition(matrices=[Tgoal], seedik=10)  # call motion planner with goal joint angles
        robot.WaitForController(0)  # wait

        #urdf_path = robot_files_path + "/" + robot_file_name + ".urdf"
        #srdf_path = robot_files_path + "/" + robot_file_name + ".srdf"
        #HandFilesGenerator(config_manipulator_name, hand_files_path, hand_file_name)

        #self.robot_name = or_utils.load_robot(urdf_path, srdf_path)
        #if self.robot_name is None:
        #    return

        #self.robot = self.env.GetRobot(self.robot_name)
        raw_input("key to continue")
        #self.read_manipulator_config()

if __name__ == '__main__':
    rospy.init_node("openrave_planner_benchmark")

    OpenRAVEPlanner()
