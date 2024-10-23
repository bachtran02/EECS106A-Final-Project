#! /usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from intera_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

def user_input():
    """
    (x, y, z) = (0.6, 0, 0.3)
                (0.7, 0, 0.5)
                (0.5, -0.3, 0.4)
    """

    x = float(input('Enter x: '))
    y = float(input('Enter y: '))
    z = float(input('Enter z: '))
    return x, y, z

def ik_service_client(x, y, z):
    service_name = "ExternalTools/right/PositionKinematicsNode/IKService"
    ik_service_proxy = rospy.ServiceProxy(service_name, SolvePositionIK)
    ik_request = SolvePositionIKRequest()
    header = Header(stamp=rospy.Time.now(), frame_id='base')

    # Create a PoseStamped and specify header (specifying a header is very important!)
    pose_stamped = PoseStamped()
    pose_stamped.header = header

    # Set end effector position: YOUR CODE HERE
    pt = Point(x, y, z)
    
    # Set end effector quaternion: YOUR CODE HERE
    quat = Quaternion(0, 1, 0, 0)

    pose_stamped.pose = Pose(
        position=pt,
        orientation=quat
    )

    # Add desired pose for inverse kinematics
    ik_request.pose_stamp.append(pose_stamped)
    # Request inverse kinematics from base to "right_hand" link
    ik_request.tip_names.append('right_hand')

    rospy.loginfo("Running Simple IK Service Client example.")

    try:
        rospy.wait_for_service(service_name, 5.0)
        response = ik_service_proxy(ik_request)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Service call failed: %s" % (e,))
        return

    # Check if result valid, and type of seed ultimately used to get solution
    if (response.result_type[0] > 0):
        rospy.loginfo("SUCCESS!")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(list(zip(response.joints[0].name, response.joints[0].position)))
        rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
        rospy.loginfo("------------------")
        rospy.loginfo("Response Message:\n%s", response)
    else:
        rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
        rospy.logerr("Result Error %d", response.result_type[0])
        return False

    return True


def main():
    rospy.init_node("ik_service_client")

    x, y, z = user_input()

    ik_service_client(x, y, z)

if __name__ == '__main__':
    main()