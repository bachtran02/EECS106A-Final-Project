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

        # TODO: check for errors
        # group = MoveGroupCommander(group_name, wait_for_servers=10)
        # Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        # set velocity
        # group.set_max_velocity_scaling_factor(0.4)  # 50% of max velocity
        # group.set_max_acceleration_scaling_factor(0.4)  # 50% of max acceleration

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

def move_arm_probe(points: list, group_name='right_arm', link='_gripper_tip', source_frame='base', paused=True):
    # offset wrist and gripper_tip: -0.27

    global button_pressed

    rospy.wait_for_service('compute_ik')
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    rospy.Subscriber('button_topic', Bool, button_callback)

    points_with_z = []

    base_request = GetPositionIKRequest()
    base_request.ik_request.group_name = group_name
    base_request.ik_request.ik_link_name = link
    base_request.ik_request.pose_stamped.header.frame_id = source_frame
    # base_request.ik_request.attempts = 20

    base_request.ik_request.pose_stamped.pose.orientation.x = 0.0
    base_request.ik_request.pose_stamped.pose.orientation.y = 1.0
    base_request.ik_request.pose_stamped.pose.orientation.z = 0.0
    base_request.ik_request.pose_stamped.pose.orientation.w = 0.0

    # constraints
    joint_constraint_j0 = JointConstraint()
    joint_constraint_j0.joint_name = 'right_j0'
    joint_constraint_j0.position = 0.0
    joint_constraint_j0.tolerance_above = 0.5
    joint_constraint_j0.tolerance_below = 0.5
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
        move([x, y, z], base_request, compute_ik, constraints, group_name='right_arm', paused=False)

        # ==============================================

        request = GetPositionIKRequest()
        request.ik_request.group_name = group_name
        request.ik_request.ik_link_name = link
        request.ik_request.pose_stamped.header.frame_id = source_frame
        # request.ik_request.attempts = 20

        # Set the desired orientation for the end effector
        request.ik_request.pose_stamped.pose.position.x = x
        request.ik_request.pose_stamped.pose.position.y = y
        # request.ik_request.pose_stamped.pose.position.z -= step_down_dist
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
                print(points_with_z[-1])
                button_pressed = False
                # move back up
                request.ik_request.pose_stamped.pose.position.z = 0.1
                try:

                    # Send the request to the service
                    response = compute_ik(request)

                    # TODO: check for errors
                    group = MoveGroupCommander(group_name)
                    
                    group.set_pose_target(request.ik_request.pose_stamped)

                    # Plan IK
                    plan = group.plan()
                    group.execute(plan[1])
                    #rate.sleep()
                except rospy.ServiceException as e:
                    print("Service call failed: %s"%e)

                break

            try:

                # Send the request to the service
                response = compute_ik(request)

                # TODO: check for errors
                group = MoveGroupCommander(group_name)
                
                group.set_pose_target(request.ik_request.pose_stamped)

                # Plan IK
                plan = group.plan()
                group.execute(plan[1])
                #rate.sleep()
                

            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
        
    print(points_with_z)
    return points_with_z

    

def move_arm_plot(points: list, group_name='right_arm', link='right_hand', source_frame='base', paused=True):
    """
    Note: If a Sawyer does not have a gripper,
    replace '_gripper_tip' with '_wrist' instead
    """

    # TODO: add timeout here
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    move_group = MoveGroupCommander(group_name, wait_for_servers=10)

    base_request = GetPositionIKRequest()
    base_request.ik_request.group_name = group_name
    base_request.ik_request.ik_link_name = link
    base_request.ik_request.pose_stamped.header.frame_id = source_frame
    # base_request.ik_request.attempts = 20

    base_request.ik_request.pose_stamped.pose.orientation.x = 0.0
    base_request.ik_request.pose_stamped.pose.orientation.y = 1.0
    base_request.ik_request.pose_stamped.pose.orientation.z = 0.0
    base_request.ik_request.pose_stamped.pose.orientation.w = 0.0

    # constraints
    joint_constraint_j0 = JointConstraint()
    joint_constraint_j0.joint_name = 'right_j0'
    joint_constraint_j0.position = 0.0
    joint_constraint_j0.tolerance_above = 0.5
    joint_constraint_j0.tolerance_below = 0.5
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
        x, y, z = point[0], point[1], -0.03

        # move to (x, y) position to plot
        move([x, y, z], base_request, compute_ik, constraints, move_group, paused=False)

        # ============================================ #

        x, y, z = point[0], point[1], -0.084
        #x, y, z = point[0], point[1], 0.02

        move([x, y, z], base_request, compute_ik, constraints, move_group, paused=False)
    
        # ==================================

        x, y, z = point[0], point[1], -0.03
        move([x, y, z], base_request, compute_ik, constraints, move_group, paused=False)

        # # Now add a collision object
        # planning_scene_interface = PlanningSceneInterface()

        # # Create and define a collision object (e.g., a box)
        # collision_object = CollisionObject()
        # collision_object.id = 'obstacle_table'  # Name the object

        # # Define the position and shape of the collision object (a box)
        # table_pose = PoseStamped()
        # table_pose.header.frame_id = "base"  # You can use "world" or another frame like "base_link"
        # table_pose.header.stamp = rospy.Time.now()  # Timestamp
        # table_pose.pose.position.x = 0.0  # x-coordinate of the table
        # table_pose.pose.position.y = 0.0  # y-coordinate of the table
        # table_pose.pose.position.z = -0.095  # z-coordinate of the table
        # table_pose.pose.orientation.w = 1.0  # Orientation (no rotation)

        # # Define the dimensions of the collision object (table dimensions)
        # table_dimensions = (3, 3, 0.001)  # Length, width, height in meters

        # # Add the collision object (table) to the planning scene
        # planning_scene_interface.add_box(collision_object.id, table_pose, table_dimensions)