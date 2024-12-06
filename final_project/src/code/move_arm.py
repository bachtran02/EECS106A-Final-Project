import roslaunch
import rospkg
import rospy

from moveit_commander import MoveGroupCommander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse

def tuck_arm():
    """
    Tuck the robot arm to the start position. Use with caution
    """
    if input('Would you like to tuck the arm? (y/n): ') == 'y':
        rospack = rospkg.RosPack()
        path = rospack.get_path('final_project')
        launch_path = path + '/launch/custom_tuck.launch'
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
        launch.start()
    else:
        print('Canceled. Not tucking the arm.')

def move_arm(points: list, group_name='right_arm', link='_gripper_tip', source_frame='base', paused=True):
    """
    Note: If a Sawyer does not have a gripper,
    replace '_gripper_tip' with '_wrist' instead
    """

    # TODO: add timeout here
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    for point in points:

        x, y, z = point[0], point[1], 0.0

        request = GetPositionIKRequest()
        request.ik_request.group_name = group_name
        request.ik_request.ik_link_name = link
        request.ik_request.pose_stamped.header.frame_id = source_frame
        # request.ik_request.attempts = 20

        # Set the desired orientation for the end effector
        request.ik_request.pose_stamped.pose.position.x = x
        request.ik_request.pose_stamped.pose.position.y = y
        request.ik_request.pose_stamped.pose.position.z = z
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:

            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response
            print('IK Reponse:\n', response)

            # TODO: check for errors
            group = MoveGroupCommander(group_name)
            # Setting position and orientation target
            # NOTE: We can set the position without specifying orientation
            # group.set_position_target([0.5, 0.5, 0.0])
            group.set_pose_target(request.ik_request.pose_stamped)

            # Plan IK
            plan = group.plan()

            if paused:
                user_input = input('Enter \'y\' if the trajectory looks safe on RVIZ: ')
                if user_input == 'y':
                    group.execute(plan[1])
            else: 
                group.execute(plan[1])
                rospy.sleep(1.0)

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)