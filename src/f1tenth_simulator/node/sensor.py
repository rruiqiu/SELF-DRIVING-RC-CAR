#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def publisher():
    # Initialize the node
    rospy.init_node('sensor')
    
    # Create a publisher for the new topic
    pub = rospy.Publisher('/sensor_data', String, queue_size=10)
    
    # Loop and publish messages
    rate = rospy.Rate(10) # 1 Hz
    while not rospy.is_shutdown():
        message = String()
        message.data = 'Fast'
        pub.publish(message)
        rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    publisher()