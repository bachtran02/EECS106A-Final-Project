import rospy
from my_chatter.msg import TimestampString

def callback(message):
    print(f'Message: {message.user_message}, Sent at: {message.sent_time}, Received at: {rospy.get_time()}')

def listener():
    rospy.Subscriber('user_messages', TimestampString, callback)
    rospy.spin()

if __name__ == '__main__':
    print("Listener started.")
    rospy.init_node('listen', anonymous = True)
    listener()