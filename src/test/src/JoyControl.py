#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Int64
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64

def callback(data):   ## callback to send data to VESC 
    global multiplier
    global pub
    a = data.axes[4]*float(multiplier)
    pub.publish(a)
    
def select_method(cm):  ## select control method to drive VESC
    method = ["commands/motor/duty_cycle", "commands/motor/speed", "commands/motor/current"]
    try:
        return(method[cm])
    except IndexError: 
        return(method[0])
     
def JoyControl():  ## node function
    global pub
    global multiplier
    rospy.init_node("JoyControl", anonymous=True)  ## intialize node
    rospy.Subscriber("/joy", Joy, callback)  ## set subscriber
    cm = rospy.get_param("~ControlMethod")  ## read parameter values
    multiplier = rospy.get_param("~Multiplier")
    pub = rospy.Publisher(select_method(cm), Float64, queue_size=10)  ## set publisher 
    rospy.spin()  

if __name__ == '__main__':
    JoyControl()
