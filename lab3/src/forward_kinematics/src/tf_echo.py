#!/usr/bin/env python

import tf2_ros
import sys
import rospy

if __name__ == '__main__':

    rospy.init_node('tf_echo_node', anonymous=True)

    assert len(sys.argv) == 3
    target_frame, source_frame = sys.argv[1:]

    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    # r = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time())
            print(trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
        
        # r.sleep()
        

