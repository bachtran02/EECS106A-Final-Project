import rospy
from my_chatter.msg import TimestampString

def main():

    pub = rospy.Publisher('user_messages', TimestampString, queue_size=10)
    r = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():

        print('Please enster a line of text and press <Enter>:')
        user_text = input()
        msg = TimestampString()
        msg.user_message = user_text
        msg.sent_time = rospy.get_time()
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    print("Publisher started.")
    rospy.init_node('test_node', anonymous=True)

    try:
        main()
    except rospy.ROSInterruptException: pass