#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from sensor_msgs.msg import LaserScan

def callback(data):
    global pub1
    global pub2
    global pub3
    global forward_value
    front_left =(data.ranges[0:10])
    front_right=(data.ranges[-10:])
    front = front_right+front_left
    Min=min(front)
    if(Min>1):
        verd="far"
        pub1.publish(forward_value)
        pub3.publish(0.48)
    if(Min<1):
        verd="near"
        brake()
    print verd


def brake():
    global pub2
    global pub1
    brake_value = rospy.get_param("~BrakeValue")
    pub2.publish(brake_value)
    while 1:                                      # disable the car after stop
        pass

def select_method(cm):  ## select control method to drive VESC
    method = ["commands/motor/duty_cycle", "commands/motor/speed", "commands/motor/current"]
    try:
        return(method[cm])
    except IndexError: 
        return(method[0])
     

def lidardetect():
    global pub1
    global pub2
    global pub3
    global brake_value
    global forward_value
    rospy.init_node("LidarDetect", anonymous=True)
    rospy.Subscriber("/scan", LaserScan, callback)
    cm = rospy.get_param("~ControlMethod")              ## read parameter values
    forward_value = rospy.get_param("~ForwardValue")
    pub1 = rospy.Publisher(select_method(cm), Float64, queue_size=10)
    pub2 = rospy.Publisher("commands/motor/brake", Float64, queue_size=10)
    pub3 = rospy.Publisher("commands/servo/position",Float64, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    lidardetect()
