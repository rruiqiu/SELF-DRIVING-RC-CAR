#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import time

#ROS Imports
import rospy
from vision_msgs.msg import Detection2DArray


class objectDetection:
    def __init__(self):
        self.classId_timer = None # initialize the timer to None
        self.duration_threshold = 3 #detection interval of 3 seconds
        #Topics & Subs, Pubs
        # Read paramters form params.yaml
        camera_topic=rospy.get_param('~camera_topic')
    
        #Subscriptions,Publishers
        rospy.Subscriber(camera_topic,Detection2DArray, self.camera_callback,queue_size=1)

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
                print("Hairbrush has been detected for more than 3 seconds!")
                self.classId_timer = None # reset the timer
        elif (self.classId_timer is not None) and (rospy.Time.now() - self.classId_timer).to_sec() > self.duration_threshold:
            self.classId_timer = None # reset the timer when classId is not equal to 72

def main(args):
    rospy.init_node("objectDetection_node", anonymous=True)
    wf = objectDetection()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)