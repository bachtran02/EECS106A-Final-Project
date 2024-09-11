#!/usr/bin/env python
import numpy as np
import rospy
from turtle_patrol.srv import Patrol  # Import service type
import sys

def patrol_client():
    # Initialize the client node
    rospy.init_node('turtle1_patrol_client')
    # Wait until patrol service is ready
    rospy.wait_for_service('/patrol')
    
    try:
        # Acquire service proxy
        patrol_proxy = rospy.ServiceProxy(
            '/patrol', Patrol)
        print(sys.argv)
        vel = float(sys.argv[1])  # Linear velocity
        omega = float(sys.argv[2])  # Angular velocity
        x = float(sys.argv[3])
        y = float(sys.argv[4])
        theta = float(sys.argv[5])
        name = sys.argv[6]
        rospy.loginfo('Command turtle1 to patrol')
        # Call patrol service via the proxy
        patrol_proxy(name, vel, omega, x, y, theta)
    except rospy.ServiceException as e:
        rospy.loginfo(e)


if __name__ == '__main__':
    patrol_client()

