#!/usr/bin/env python

# Example to demonstrate moving to stored/names targets. Both arm and hand movements executed.
# Available named targets can be viewed in MoveIt, on the planning tab.

import rospy
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander


rospy.init_node("sr_fan_demo", anonymous=True)

hand_commander = SrHandCommander()
arm_commander = SrArmCommander()

print("Executing moves:")

# start a bit higher
# arm_commander.move_to_named_target("gamma", True)

print("Move home")
arm_commander.move_to_named_target("home_pose_both", True)
hand_commander.move_to_named_target("home_pose_both", True)
# print("Move to pick fan")
# arm_commander.move_to_named_target("move_to_pick_fan", True)

# print("Pick fan")
# hand_commander.move_to_named_target("pick_fan", False)

# print("Retreat with fan")
# arm_commander.move_to_named_target("retreat_with_fan", True)

# # Define trajectory for the hand. This will shake the fan around
# trajectory = [
#     {
#         'name': 'open_fan',
#         'interpolate_time': 1.0
#     },
#     {
#         'name': 'move_in',
#         'interpolate_time': 3.0
#     },
#     {
#         'name': 'move_out',
#         'interpolate_time': 3.0
#     },
#     {
#         'name': 'move_in',
#         'interpolate_time': 3.0
#     },
#     {
#         'name': 'move_out',
#         'interpolate_time': 3.0
#     },
#     {
#         'name': 'move_in',
#         'interpolate_time': 3.0
#     },
#     {
#         'name': 'move_out',
#         'interpolate_time': 3.0
#     }
# ]

# # Run trajectory unsafe
# print("Shaking the fan around")
# hand_commander.run_named_trajectory_unsafe(trajectory, True)
