import rospy
from std_msgs.msg import String

def main():

    pub = rospy.Publisher('user_messages', String, queue_size=10)
    r = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():

        print('Please enster a line of text and press <Enter>:')
        user_text = input()
        pub_string = f"{user_text} {rospy.get_time()}"
        pub.publish(pub_string)
        r.sleep()

if __name__ == '__main__':

    rospy.init_node('test_node', anonymous=True)

    try:
        main()
    except rospy.ROSInterruptException: pass