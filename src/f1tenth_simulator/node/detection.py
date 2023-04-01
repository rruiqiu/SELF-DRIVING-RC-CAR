#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import time

#ROS Imports
import rospy
from vision_msgs.msg import Detection2DArray
from std_msgs.msg import String

class objectDetection:
    def __init__(self):
        self.classId_timer = None # initialize the timer to None
        self.duration_threshold = 3 #detection interval of 3 seconds
    

        #Topics & Subs, Pubs
        # Read paramters form params.yaml
        camera_topic=rospy.get_param('~camera_topic')
        sensor_topic=rospy.get_param('~imu_topic')
        output_topic=rospy.get_param('~output_topic')
    
        #Subscriptions,Publishers
        rospy.Subscriber(sensor_topic,String,self.sensor_callback,queue_size=10)
        rospy.Subscriber(camera_topic,Detection2DArray, self.camera_callback,queue_size=1)
        
        self.output_pub =rospy.Publisher(output_topic,String,queue_size=5)
  
        # Initialize a flag that will indicate whether the sensor callback is paused
        self.sensor_callback_paused = False

    def getclassID(self, data):
        #we first loop through each detection in the detections array. Then, for each detection, we loop through each result in the detection.results array and gets its id and confidence level
        for i in range(0,len(data.detections)):
          for j in range(0,len(data.detections[i].results)):
            id = int(data.detections[i].results[j].id)
            level = float(data.detections[i].results[j].score)
        return id, level

    def camera_callback(self, data):
        classId,score=self.getclassID(data)
        #find the id of a stop sign 
        if classId == 72:
            if self.classId_timer is None:
                self.classId_timer = rospy.Time.now() # start the timer
            elif (rospy.Time.now() - self.classId_timer).to_sec() >= self.duration_threshold:
                rospy.loginfo('Object detected, pausing sensor callback')
                self.sensor_callback_paused = True
                start_time = rospy.Time.now()
                while (rospy.Time.now() - start_time).to_sec() < 5.0:  # publish for 5 seconds
                    self.output_pub.publish('Slow')
                    rospy.Rate(10).sleep()  # publish at a rate of 10 Hz
                    self.sensor_callback_paused = True
                self.classId_timer = None # reset the timer
                self.unpause_sensor_callback() 
        elif (self.classId_timer is not None) and (rospy.Time.now() - self.classId_timer).to_sec() > self.duration_threshold:
            self.classId_timer = None # reset the timer when classId is not equal to 72
            self.unpause_sensor_callback() 
        else:
            self.unpause_sensor_callback() 

    def sensor_callback(self,data):
        # Check if the sensor callback is paused
        if self.sensor_callback_paused is True:
            # Print a message indicating that the sensor callback is paused
            rospy.loginfo('Sensor callback paused due to object detection')
        elif self.sensor_callback_paused is False:
            self.output_pub.publish(data)
    
    def unpause_sensor_callback(self):
        # Unpause the sensor callback
        self.sensor_callback_paused = False
        
        # Print a message indicating that the sensor callback is unpaused
        rospy.loginfo('Sensor callback unpaused')    


def main(args):
    rospy.init_node("objectDetection_node", anonymous=True)
    wf = objectDetection()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)