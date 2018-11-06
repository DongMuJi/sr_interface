#!/usr/bin/python
#
# Copyright (C) 2018 Shadow Robot Company Ltd - All Rights Reserved.
# Proprietary and Confidential. Unauthorized copying of the content in this file, via any medium is strictly prohibited.

import rospy
from sr_utilities.hand_finder import HandFinder
from controller_manager_msgs.srv import ListControllers


class InitializeTeachMode(object):
    def __init__(self):
        rospy.loginfo("Initialized...")
        self._hand_finder = HandFinder()
        self._hand_e = self._hand_finder.hand_e_available()
        self._hand_h = self._hand_finder.hand_h_available()

        self.joints = list()
        self.running_controllers = list()

        list_controllers = rospy.ServiceProxy(
            'controller_manager/list_controllers', ListControllers)

        controllers_running = False
        while not controllers_running:
            try:
                controllers_info = list_controllers()
                rospy.loginfo("controllers: {}".format(controllers_info))
                controllers_running = True
            except rospy.ServiceException:
                controllers_running = False
                rospy.sleep(1)
            # except rospy.ROSInterruptException:
            #     rospy.loginfo("got exception...")
            #     exit()
            # except KeyboardInterrupt:
            #     raise
            #     rospy.loginfo("got exception...")
            #     exit()
            if not controllers_running:
                rospy.loginfo("Waiting for the controllers to start...")
        self.joints, self.running_controllers = self._get_joints(controllers_info)

        #
        # if self._hand_e:
        #     self._inti_hand_e()
        # elif self._hand_h:
        #     self._init_hand_h()
        # else:
        #     rospy.logerr("No hand was recognised")
        #     exit()

    def _get_controller_info(self, controller_info):
        rospy.logwarn("getting joints")
        total_joint_list = list()
        controller_names = list()
        for controller in controller_info.controller:
            # rospy.loginfo("controller: {}".format(controller.name))
            # rospy.loginfo("resources: {}".format(controller.claimed_resources[0].resources))
            controller_names.append(controller.name)
            total_joint_list.extend(controller.claimed_resources[0].resources)
        rospy.loginfo("controller names: {}".format(controller_names))
        return total_joint_list, controller_names

    def _init_hand_e(self):
        self.hand_controllers = {
            "effort": ["sh_{0}{1}_effort_controller".format(hand_joint_prefix, joint)
                       for joint in self.joints
                       for hand_joint_prefix in self.robot_joint_prefixes],
            "position": ["sh_{0}{1}_position_controller".format(hand_joint_prefix, joint)
                         for joint in self.joints
                         for hand_joint_prefix in self.robot_joint_prefixes],
            "mixed": ["sh_{0}{1}_mixed_position_velocity_controller".format(hand_joint_prefix, joint)
                      for joint in self.joints
                      for hand_joint_prefix in self.robot_joint_prefixes],
            "velocity": ["sh_{0}{1}_velocity_controller".format(hand_joint_prefix, joint)
                         for joint in self.joints
                         for hand_joint_prefix in self.robot_joint_prefixes],
            "stop": []}

    def _init_hand_h(self):
        self.hand_controllers = {
            "grasp": ["sh_{0}{1}_effort_controller".format(hand_joint_prefix, joint)
                       for joint in self.joints
                       for hand_joint_prefix in self.robot_joint_prefixes],
            "position": ["sh_{0}{1}_position_controller".format(hand_joint_prefix, joint)
                         for joint in self.joints
                         for hand_joint_prefix in self.robot_joint_prefixes],
            "stop": []}
        # self.joints = rospy.get_param("joints")
        # self.joints = rospy.global_name("joints")
        # self.joints = rospy.resolve_name_without_node_name("/joints")


if __name__ == "__main__":
    rospy.init_node("initialize_teach_mode", anonymous=True)

    itm = InitializeTeachMode()
    # itm.init_hand_h()
