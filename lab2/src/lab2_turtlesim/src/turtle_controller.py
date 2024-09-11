#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys

def main():

    turtlename = sys.argv[1]

    pub = rospy.Publisher(f'{turtlename}/cmd_vel', Twist, queue_size=10)
    r = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        user_text = input()
        lx, ly, lz = 0, 0, 0
        ax, ay, az = 0, 0, 0

        if user_text == "w":
            lx = 1
        if user_text == "s":
            lx = -1
        if user_text == "d":
            az= -1
        if user_text == "a":
            az = 1

        
        message = Twist()
        # message.linear.x, message.linear.y, message.linear.z = lx, ly, lz
        # message.angular.x, message.angular.y, message.angular.z = ax, ay, az
        message.linear.x = lx
        message.angular.z = az

        pub.publish(message)

        r.sleep()

if __name__ == '__main__':
    print("Publisher started.")
    rospy.init_node('test_node', anonymous=True)

    try:
        main()
    except rospy.ROSInterruptException: pass