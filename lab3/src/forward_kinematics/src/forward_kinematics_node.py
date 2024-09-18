#!/usr/bin/env python
# The line above tells Linux that this file is a Python script, and that the OS
# should use the Python interpreter in /usr/bin/env to run it. Don't forget to
# use "chmod +x [filename]" to make this script executable.

# Import the dependencies as described in example_pub.py
import rospy
# from std_msgs.msg import String
from sensor_msgs.msg import JointState

from forward_kinematics import baxter_forward_kinematics_from_joint_state

def callback(joinState: JointState):
    baxter_forward_kinematics_from_joint_state(joinState)

# Define the method which contains the node's main functionality
def listener():

    rospy.Subscriber("robot/joint_states", JointState, callback)

    # Wait for messages to arrive on the subscribed topics, and exit the node
    # when it is killed with Ctrl+C
    rospy.spin()


# Python's syntax for a main() method
if __name__ == '__main__':

    rospy.init_node('fw_node', anonymous=True)

    listener()
