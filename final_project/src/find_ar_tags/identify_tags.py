#!/usr/bin/env python

import sys

import cv2
from PIL import Image

import rospy
import rospkg
import roslaunch

import matplotlib.pyplot as plt
import numpy as np

import tf2_ros

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
from numpy import linalg
from intera_interface import gripper as robot_gripper

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from intera_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

def tuck():
    """
    Tuck the robot arm to the start position. Use with caution
    """
    if input('Would you like to tuck the arm? (y/n): ') == 'y':
        rospack = rospkg.RosPack()
        path = rospack.get_path('final_project')
        launch_path = path + '/custom_tuck/launch/custom_tuck.launch'
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
        launch.start()
    else:
        print('Canceled. Not tucking the arm.')

def lookup_tag(tag_number):
    """
    Given an AR tag number, this returns the position of the AR tag in the robot's base frame.
    You can use either this function or try starting the scripts/tag_pub.py script.  More info
    about that script is in that file.

    Parameters
    ----------
    tag_number : int

    Returns
    -------
    3x' :obj:`numpy.ndarray`
        tag position
    """
    
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    ar_tag_string = "ar_marker_{}".format(tag_number)

    retry = 0
    while True:

        try:
            # The rospy.Time(0) is the latest available 
            # The rospy.Duration(10.0) is the amount of time to wait for the transform to be available before throwing an exception
            trans = tfBuffer.lookup_transform("base", ar_tag_string, rospy.Time(0), rospy.Duration(10.0))
            break
        except Exception as e:
            print(f'Failed to find {ar_tag_string}: err={e}')
        retry += 1
        
        if retry == 5:
            sys.exit(1)
        
        print("Retrying...")

    tag_pos = [getattr(trans.transform.translation, dim) for dim in ('x', 'y', 'z')]
    return np.array(tag_pos)


def create_rectangle_dots(x1, y1, x2, y2, x3, y3, image_path, offset=0.5, dot_dist=0.1):
    x4 = x2 + (x3 - x1)
    y4 = y2 + (y3 - y1)

    x12 = x1 - x2
    y12 = y1 - y2
    x13 = x1 - x3
    y13 = y1 - y3
    v12 = np.sqrt((x12)**2 + (y12)**2)
    v13 = np.sqrt((x13)**2 + (y13)**2)

    # Swap the sides if necessary, ensuring (x1, y1) to (x2, y2) is the longer side
    if v12 < v13:
        x2, y2, x3, y3 = x3, y3, x2, y2
        x12 = x1 - x2
        y12 = y1 - y2
        x13 = x1 - x3
        y13 = y1 - y3
        v12 = np.sqrt((x12)**2 + (y12)**2)
        v13 = np.sqrt((x13)**2 + (y13)**2)


    x1_ = x1 - offset*x12/v12
    y1_ = y1 - offset*y12/v12
    x2_ = x2 + offset*x12/v12
    y2_ = y2 + offset*y12/v12

    xp1 = x1_ - offset*x13/v13
    yp1 = y1_ - offset*y13/v13
    xp2 = x2_ - offset*x13/v13
    yp2 = y2_ - offset*y13/v13
    xp3 = (x1_ - x13) + offset*x13/v13
    yp3 = (y1_ - y13) + offset*y13/v13
    xp4 = (x2_ - x13) + offset*x13/v13
    yp4 = (y2_ - y13) + offset*y13/v13

    xp12 = xp1 - xp2
    yp12 = yp1 - yp2
    xp13 = xp1 - xp3
    yp13 = yp1 - yp3
    vp12 = np.sqrt((xp12)**2 + (yp12)**2)
    vp13 = np.sqrt((xp13)**2 + (yp13)**2)
    rows = np.floor(vp12/dot_dist)
    cols = np.floor(vp13/dot_dist)

    dithered_image = process_image(image_path, rows, cols)
    image_size = dithered_image._size
    cur_col = image_size[1]
    cur_row = image_size[0]

    #print_ascii_art(dithered_image)
    binary_array = (np.array(dithered_image) == 0).astype(int)

    new_vp12 = dot_dist * (cur_row - 1)
    new_vp13 = dot_dist * (cur_col - 1)

    xp2 = xp1 - new_vp12*xp12/vp12
    yp2 = yp1 - new_vp12*yp12/vp12
    xp3 = xp1 - new_vp13*xp13/vp13
    yp3 = yp1 - new_vp13*yp13/vp13
    xp4 = xp2 + xp3 - xp1
    yp4 = yp2 + yp3 - yp1

    d2x = (xp2-xp1)/(cur_row - 1) # horizontal small step from p1 to p2
    d2y = (yp2-yp1)/(cur_col - 1) # vertical small step from p1 to p2
    d3x = (xp3-xp1)/(cur_row - 1) # horizontal small step from p1 to p3
    d3y = (yp3-yp1)/(cur_col - 1) # vertical small step from p1 to p3
    grid = []

    for i in range(cur_row):
        for j in range(cur_col):
            x = xp1 + j * d2x + i * d3x
            y = yp1 + j * d2y + i * d3y
            grid.append([x, y])

    pts_to_plot = []
    for i in range(len(binary_array)):
        for j in range(len(binary_array[0])):
            if binary_array[i][j] == 1:
                pts_to_plot.append(grid[i * (len(binary_array[0])) + j])

    x_coords, y_coords = zip(*pts_to_plot)
    # plt.scatter(x_coords, y_coords, color='black', label='Points')  # Plot points
    # plt.scatter([x1, x2, x3, x4], [y1, y2, y3, y4], color='red', marker='o')
    # plt.scatter([xp1, xp2, xp3, xp4], [yp1, yp2, yp3, yp4], color='green', marker='o')
    # plt.grid(True)  # Add gridlines for better visualization
    # plt.show()

    # pts_to_plot = [[x1, y1], [x2, y2], [x3, y3], [x4, y4]]
    return pts_to_plot

def process_image(image_path, canvas_width, canvas_height):
    # Load the image
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    
    # Get original dimensions and compute aspect ratio
    original_height, original_width = img.shape
    aspect_ratio = original_width / original_height

    # Calculate new dimensions while maintaining aspect ratio
    if canvas_width / canvas_height > aspect_ratio:
        new_height = int(canvas_height)
        new_width = int(aspect_ratio * canvas_height)
    else:
        new_width = int(canvas_width)
        new_height = int(canvas_width / aspect_ratio)
    
    # Resize the image to fit within the canvas
    resized_img = cv2.resize(img, (new_width, new_height), interpolation=cv2.INTER_AREA)
    
    # Convert to a PIL image and apply Floyd-Steinberg dithering
    pil_image = Image.fromarray(resized_img)
    dithered_image = pil_image.convert("1")  # Convert to black-and-white using dithering

    return dithered_image

def print_ascii_art(image):
    # Convert the image to a binary array (0 for white, 1 for black)
    binary_array = (np.array(image) == 0).astype(int)  # 0 is black in "1" mode

    # Generate the ASCII art
    ascii_art = "\n".join("".join("#" if pixel == 1 else " " for pixel in row) for row in binary_array)

    # Print the ASCII art
    print(ascii_art)

def rotate_point(x, y, angle, origin=(0, 0)):
    """Rotate a point (x, y) around a given origin by a specified angle (in radians)."""
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)
    x_rot = cos_angle * (x - origin[0]) - sin_angle * (y - origin[1]) + origin[0]
    y_rot = sin_angle * (x - origin[0]) + cos_angle * (y - origin[1]) + origin[1]
    return x_rot, y_rot

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
    """
    python find_ar_tags/identify_tags.py
    """

    # ar_tags = [11, 0, 15]
    ar_tags = [1, 2, 5]

    rospy.init_node('find_tags_node')
    
    # move the camera to correct position
    tuck()

    # Lookup the AR tag position.
    tags_pos = [lookup_tag(ar_tag) for ar_tag in ar_tags]
    
    # ensure that all AR tags are found
    assert len(tags_pos) == len(ar_tags)

    x1, y1 = tags_pos[0][0], tags_pos[0][1]
    x2, y2 = tags_pos[1][0], tags_pos[1][1]
    x3, y3 = tags_pos[2][0], tags_pos[2][1]

    #x1, y1 = 0, 1
    #x2, y2 = 5, 1
    #x3, y3 = 0, 5

    #x1, y1 = 3, 3
    #x2, y2 = -1, 5
    #x3, y3 = 2, 1

    #x1, y1 = rotate_point(1, (8.5/11), 0.05)
    #x2, y2 = rotate_point(1, 0, 0.05)
    #x3, y3 = rotate_point(0, (8.5/11), 0.05)

    image_path = "../../assets/img/bw_smiley.jpg"
    painting_points = create_rectangle_dots(x1, y1, x2, y2, x3, y3, image_path, offset=0.08, dot_dist=0.01)
    # moving(painting_points)

    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

    # while not rospy.is_shutdown():
    #     input('Press [ Enter ]: ')

    # right_gripper = robot_gripper.Gripper('right_gripper')

    for point in painting_points:
            
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        #link = "stp_022312TP99620_tip_1"
        link = "_gripper_tip"

        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        

        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = point[0]
        request.ik_request.pose_stamped.pose.position.y = point[1]
        request.ik_request.pose_stamped.pose.position.z = 0.0     
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:

            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print('Reponse:\n', response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # # Setting just the position without specifying the orientation
            # group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK
            plan = group.plan()
            # user_input = input("Enter 'y' if the trajectory looks safe on RVIZ: ")  
            # # Execute IK if safe
            # if user_input == 'y':
            #     group.execute(plan[1])

            group.execute(plan[1])
            rospy.sleep(1.0)
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
    

if __name__ == "__main__":
    main()