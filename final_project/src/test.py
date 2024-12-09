#! /usr/bin/env python
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import Pose
from std_msgs.msg import String

def move_arm_downward():
    # Initialize MoveIt! Commander
    rospy.init_node('test_node', anonymous=True)

    # Initialize RobotCommander and MoveGroupCommander for the right arm
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    group = MoveGroupCommander("right_arm")

    # Get the current pose of the arm
    current_pose = group.get_current_pose().pose
    rospy.loginfo(f"Current Pose: {current_pose}")

    # Modify the current position to move the arm downward
    # Assuming we're moving along the Z-axis (downward)
    target_pose = Pose()
    # target_pose.position.x = current_pose.position.x  # Keep x position the same
    # target_pose.position.y = current_pose.position.y  # Keep y position the same
    # target_pose.position.z = current_pose.position.z - 0.1  # Move 0.1 meters downward
    target_pose.orientation.x = current_pose.orientation  # Keep the same orientation

    target_pose.orientation.x = 0.0
    target_pose.orientation.y = 1.0
    target_pose.orientation.z = 0.0
    target_pose.orientation.w = 0.0

    # Set the target pose for the move group
    group.set_pose_target(target_pose)

    # Plan and execute the motion
    plan = group.go(wait=True)

    # Optionally, ensure the arm reached the target pose
    if plan:
        rospy.loginfo("Arm moved downward successfully.")
    else:
        rospy.logwarn("Failed to move arm downward.")

if __name__ == "__main__":
    # try:
    #     move_arm_downward()
    # except rospy.ROSInterruptException:
    #     pass

    from moveit_commander import RobotCommander

    # Initialize MoveIt! Commander
    robot_commander = RobotCommander()

    # Get the joint names for the robot
    joint_names = robot_commander.get_group('right_arm').get_joints()

    print(joint_names)
