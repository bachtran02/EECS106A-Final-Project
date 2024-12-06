#! /usr/bin/env python
# from moveit_commander import MoveGroupCommander

# group = MoveGroupCommander("right_arm")

# # Get all links in the group
# links = group.get_joints()
# print("Links in the 'right_arm' group:")
# for link in links:
#     print(link)

# # Get the end-effector link
# end_effector = group.get_end_effector_link()
# print("End-effector link:", end_effector)

# from moveit_commander import RobotCommander

# # Initialize the RobotCommander
# robot = RobotCommander()

# # Get all available move groups
# move_groups = robot.get_group_names()

# # Print available groups
# print("Available MoveIt! groups:")
# for group in move_groups:
#     print(group)

from moveit_commander import MoveGroupCommander
import time

# Initialize the move group for the right arm
group = MoveGroupCommander("right_arm")

# Check if connection is established
if not group._g.is_connected():
    print("Failed to connect to move_group action server!")
else:
    print("Successfully connected to move_group action server!")
