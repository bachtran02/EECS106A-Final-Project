#!/usr/bin/env python

import sys

import rospy
import rospkg
import roslaunch

import numpy as np

import tf2_ros

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

def main():
    """
    python find_ar_tags/identify_tags.py
    """

    # ar_tags = [1, 2, 3, 4]
    ar_tags = [11, 16]
    
    rospy.init_node('find_tags_node')
    
    # move the camera to correct position
    # tuck()

    # Lookup the AR tag position.
    tags_pos = [lookup_tag(ar_tag) for ar_tag in ar_tags]
    
    # ensure that all AR tags are found
    assert len(tags_pos) == len(ar_tags)

    print(tags_pos)


if __name__ == "__main__":
    main()