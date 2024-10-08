#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def hello_publisher():
    # Initialize the node with a name 'hello_publisher'
    rospy.init_node('hello_publisher', anonymous=True)

    # Create a publisher object, publishing String messages to the topic 'hello_jeff'
    pub = rospy.Publisher('hello_jeff', String, queue_size=10)

    # Define the rate of publishing messages (1 Hz)
    rate = rospy.Rate(5)  # 1 message per second

    while not rospy.is_shutdown():
        hello_str = "hello"
        rospy.loginfo(hello_str)  # Print the message to the console
        pub.publish(hello_str)  # Publish the message to the topic 'hello_jeff'
        rate.sleep()  # Sleep for the remaining time to enforce the rate

if __name__ == '__main__':
    try:
        hello_publisher()
    except rospy.ROSInterruptException:
        pass

