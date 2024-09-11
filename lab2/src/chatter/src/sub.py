import rospy
from std_msgs.msg import String

def callback(message):
    message = message.data
    sent_message, sent_time = message.split(" ")
    print(f'Message: {sent_message}, Sent at: {sent_time}, Received at: {rospy.get_time()}')

def listener():
    rospy.Subscriber('user_messages', String, callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('listen', anonymous = True)
    listener()