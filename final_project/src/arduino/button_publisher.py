#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
import serial  # To handle serial communication with Arduino

def button_publisher():
    # Initialize ROS node
    rospy.init_node('button_publisher_node', anonymous=True)
    pub = rospy.Publisher('/button_topic', Bool, queue_size=10)

    # Initialize serial communication (adjust the port to match your setup)
    arduino = serial.Serial('/dev/ttyACM1', 9600, timeout=1)
    arduino.flush()

    rate = rospy.Rate(10)  # 10 Hz publishing rate
    
    while not rospy.is_shutdown():
        if arduino.in_waiting > 0:             # Check if data is available
            line = arduino.readline().decode('utf-8').strip()
            if line == "pressed":
                rospy.loginfo("Button pressed")
                pub.publish(True)              # Publish 'True' when button is pressed
            else:
                pub.publish(False)
        rate.sleep()

if __name__ == '__main__':
    try:
        button_publisher()
    except rospy.ROSInterruptException:
        pass