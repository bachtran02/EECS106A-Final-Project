import sys
import numpy as np
import rospy
import tf2_ros

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