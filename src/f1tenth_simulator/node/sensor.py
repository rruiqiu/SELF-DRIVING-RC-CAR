#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('/sensor/sensor_data', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    #rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        speed_str = "Velocity: Fast %s"
        pub.publish(speed_str)
        #rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass