#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from intera_core_msgs.srv import SolvePositionFK, SolvePositionFKRequest

"""
-0.0513480219767665, -0.9910556525678347, -0.32402473824453004, 1.6894133583031432, 0.22845921825659607, 0.9061592705066284, 1.2733730740832716
-0.1969010001175796, -1.022866728303493, -0.06724243165016237, 1.0959200348205453, 0.03841661791639141, 1.4956552870862063, 1.4831054488420887
-0.5394137475438593, -1.1089019897552566, -0.4144077131486655, 1.591428150298846, 0.2048638762486326, 1.128760403385888, 0.7371913415779361
"""

def fk_service_client():
    service_name = "ExternalTools/right/PositionKinematicsNode/FKService"
    fk_service_proxy = rospy.ServiceProxy(service_name, SolvePositionFK)
    fk_request = SolvePositionFKRequest()
    joints = JointState()
    
    # YOUR CODE HERE
    joints_str = input('Enter joints: ')
    joint_input = joints_str.split(',')
    
    assert len(joint_input) == 7

    for i in range(len(joint_input)):
        joint_input[i] = float(joint_input[i])

    joints.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3','right_j4', 'right_j5', 'right_j6']
    joints.position = joint_input
    # Add desired pose for forward kinematics
    fk_request.configuration.append(joints)
    # Request forward kinematics from base to "right_hand" link
    fk_request.tip_names.append('right_hand')

    try:
        rospy.wait_for_service(service_name, 5.0)
        response = fk_service_proxy(fk_request)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 

    # Check if result valid
    if (response.isValid[0]):
        rospy.loginfo("SUCCESS - Valid Cartesian Solution Found")
        rospy.loginfo("\nFK Cartesian Solution:\n")
        rospy.loginfo("------------------")
        rospy.loginfo("Response Message:\n%s", response)
    else:
        rospy.logerr("INVALID JOINTS - No Cartesian Solution Found.")
        return 

    return True


def main():
    rospy.init_node("fk_service_client")

    fk_service_client()

if __name__ == '__main__':
    main()
