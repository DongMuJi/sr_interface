#!/usr/bin/env python
 
import rospy
from sr_robot_commander.sr_arm_commander import SrArmCommander
from sr_robot_commander.sr_hand_commander import SrHandCommander
 
# Change name "script_demo_template_node" with name of your demo
rospy.init_node("script_demo_template_node", anonymous=True)
 
hand_commander = SrHandCommander()
arm_commander = SrArmCommander()
 
"""
Insert here demo code
"""
 
"""
Example of code to move the arm to a saved state
arm_commander.move_to_named_target("name_of_saved_state", True)
"""
arm_commander.move_to_named_target("home_pose", True)
 
"""
Example of moving only hand to a saved state without planning
for smooth movement.
trajectory = [
    {
        'name': 'name_of_saved_state',
        'interpolate_time': time to execute the movement in seconds
    }
]
hand_commander.run_named_trajectory_unsafe('trajectory', True)
"""
 
# Define the trajectory
trajectory_pack_hand = [
    {
        'name': 'pack_hand',
        'interpolate_time': 3.0
    }
]
 
# Run the trajectory
hand_commander.run_named_trajectory_unsafe(trajectory_pack_hand, True)
 
# Define the trajectory
trajectory_open_close_hand = [
    {
        'name': 'open_hand',
        'interpolate_time': 3.0
    },
    {
        'name': 'close_hand',
        'interpolate_time': 3.0
    },
    {
        'name': 'open_hand',
        'interpolate_time': 3.0
    },
    {
        'name': 'close_hand',
        'interpolate_time': 3.0
    }
]
 
# Run the trajectory
hand_commander.run_named_trajectory_unsafe(trajectory_open_close_hand, True)