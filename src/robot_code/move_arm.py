import roslaunch
import rospkg
import rospy

from std_msgs.msg import Bool, Float64

from moveit_commander import MoveGroupCommander, PlanningSceneInterface, CollisionObject
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from moveit_msgs.msg import JointConstraint, PositionConstraint, Constraints
from geometry_msgs.msg import PoseStamped


button_pressed = False

def button_callback(msg):
    global button_pressed
    button_pressed = True

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

def move(point: list, request, compute_ik, constraints, group, paused=False):
    
    assert len(point) == 3

    x, y, z = point

    request.ik_request.constraints = constraints

    # Set the desired orientation for the end effector
    request.ik_request.pose_stamped.pose.position.x = x
    request.ik_request.pose_stamped.pose.position.y = y
    request.ik_request.pose_stamped.pose.position.z = z
    
    try:

        # Send the request to the service
        response = compute_ik(request)

        # Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        # Plan IK
        plan = group.plan()

        if paused:
            user_input = input('Enter \'y\' if the trajectory looks safe on RVIZ: ')
            if user_input == 'y':
                group.execute(plan[1])
        else: 
            group.execute(plan[1])

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def move_arm_probe(points: list, group_name='right_arm', link='_gripper_tip', source_frame='base', paused=True):

    global button_pressed

    rospy.wait_for_service('compute_ik')
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    rospy.Subscriber('button_topic', Bool, button_callback)

    move_group = MoveGroupCommander(group_name, wait_for_servers=10)

    points_with_z = []

    base_request = GetPositionIKRequest()
    base_request.ik_request.group_name = group_name
    base_request.ik_request.ik_link_name = link
    base_request.ik_request.pose_stamped.header.frame_id = source_frame

    base_request.ik_request.pose_stamped.pose.orientation.x = 0.0
    base_request.ik_request.pose_stamped.pose.orientation.y = 1.0
    base_request.ik_request.pose_stamped.pose.orientation.z = 0.0
    base_request.ik_request.pose_stamped.pose.orientation.w = 0.0

    # constraints
    joint_constraint_j0 = JointConstraint()
    joint_constraint_j0.joint_name = 'right_j0'
    joint_constraint_j0.position = 0.0
    joint_constraint_j0.tolerance_above = 0.7
    joint_constraint_j0.tolerance_below = 0.7
    joint_constraint_j0.weight = 0.7

    joint_constraint_j2 = JointConstraint()
    joint_constraint_j2.joint_name = 'right_j2'
    joint_constraint_j2.position = 0.0
    joint_constraint_j2.tolerance_above = 0.4
    joint_constraint_j2.tolerance_below = 0.4
    joint_constraint_j2.weight = 0.7

    joint_constraint_j4 = JointConstraint()
    joint_constraint_j4.joint_name = 'right_j4'
    joint_constraint_j4.position = 0.0
    joint_constraint_j4.tolerance_above = 0.4
    joint_constraint_j4.tolerance_below = 0.4
    joint_constraint_j4.weight = 0.7

    constraints = Constraints()
    constraints.name = 'sawyer_constraints'
    constraints.joint_constraints= [joint_constraint_j0, joint_constraint_j2, joint_constraint_j4]

    for point in points:

        x, y, z = point[0], point[1], 0.1
        move([x, y, z], base_request, compute_ik, constraints, move_group, paused=False)

        # ==============================================

        request = GetPositionIKRequest()
        request.ik_request.group_name = group_name
        request.ik_request.ik_link_name = link
        request.ik_request.pose_stamped.header.frame_id = source_frame

        # Set the desired orientation for the end effector
        request.ik_request.pose_stamped.pose.position.x = x
        request.ik_request.pose_stamped.pose.position.y = y
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0

        z = 0
        step_down_dist = 0.004

        while True:
            
            request.ik_request.pose_stamped.pose.position.z = z
            z -= step_down_dist

            request.ik_request.constraints = constraints

            if button_pressed == True:
                # record z position
                points_with_z.append([x, y, z])
                button_pressed = False
                # move back up
                request.ik_request.pose_stamped.pose.position.z = 0.1
                try:

                    # Send the request to the service
                    response = compute_ik(request)
                    
                    move_group.set_pose_target(request.ik_request.pose_stamped)

                    # Plan IK
                    plan = move_group.plan()
                    move_group.execute(plan[1])

                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)

                break

            try:

                # Send the request to the service
                response = compute_ik(request)
                
                move_group.set_pose_target(request.ik_request.pose_stamped)

                # Plan IK
                plan = move_group.plan()
                move_group.execute(plan[1])                

            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        
    return points_with_z

    

def move_arm_plot(points: list, group_name='right_arm', link='right_hand', source_frame='base', paused=True):
    """
    Note: If a Sawyer does not have a gripper,
    replace '_gripper_tip' with '_wrist' instead
    """

    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    move_group = MoveGroupCommander(group_name, wait_for_servers=10)

    base_request = GetPositionIKRequest()
    base_request.ik_request.group_name = group_name
    base_request.ik_request.ik_link_name = link
    base_request.ik_request.pose_stamped.header.frame_id = source_frame

    base_request.ik_request.pose_stamped.pose.orientation.x = 0.0
    base_request.ik_request.pose_stamped.pose.orientation.y = 1.0
    base_request.ik_request.pose_stamped.pose.orientation.z = 0.0
    base_request.ik_request.pose_stamped.pose.orientation.w = 0.0

    # constraints
    joint_constraint_j0 = JointConstraint()
    joint_constraint_j0.joint_name = 'right_j0'
    joint_constraint_j0.position = 0.0
    joint_constraint_j0.tolerance_above = 0.7
    joint_constraint_j0.tolerance_below = 0.7
    joint_constraint_j0.weight = 0.7

    joint_constraint_j2 = JointConstraint()
    joint_constraint_j2.joint_name = 'right_j2'
    joint_constraint_j2.position = 0.0
    joint_constraint_j2.tolerance_above = 0.4
    joint_constraint_j2.tolerance_below = 0.4
    joint_constraint_j2.weight = 0.7

    joint_constraint_j4 = JointConstraint()
    joint_constraint_j4.joint_name = 'right_j4'
    joint_constraint_j4.position = 0.0
    joint_constraint_j4.tolerance_above = 0.4
    joint_constraint_j4.tolerance_below = 0.4
    joint_constraint_j4.weight = 0.7

    constraints = Constraints()
    constraints.name = 'sawyer_constraints'
    constraints.joint_constraints= [joint_constraint_j0, joint_constraint_j2, joint_constraint_j4]

    init_point = [points[0][0], points[0][1], 0.2]
    move(init_point, base_request, compute_ik, constraints, move_group, paused=False)
    
    for point in points:
        x, y, z = point[0], point[1], 0.1

        # move to (x, y) position to plot
        move([x, y, z], base_request, compute_ik, constraints, move_group, paused=False)

        # ============================================ #

        x, y, z = point[0], point[1], point[2]
        
        # Add a little offset between button height and marker height
        z += 0.001

        move([x, y, z], base_request, compute_ik, constraints, move_group, paused=False)
    
        # ==================================

        x, y, z = point[0], point[1], 0.1
        move([x, y, z], base_request, compute_ik, constraints, move_group, paused=False)
