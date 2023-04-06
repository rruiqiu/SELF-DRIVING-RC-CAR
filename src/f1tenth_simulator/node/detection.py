#!/usr/bin/env python
import rospy
from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput
from std_msgs.msg import Int32

# initialize ROS node and publisher
rospy.init_node('detection')
pub = rospy.Publisher('object_detection', Int32, queue_size=10)

# initialize detectNet, videoSource, and videoOutput
net = detectNet("ssd-mobilenet-v2", threshold=0.5)
camera = videoSource("v4l2:///dev/video2")
display = videoOutput("display://0")

while display.IsStreaming():
	img = camera.Capture()
	detections = net.Detect(img)
	display.Render(img)
	for detection in detections:
		pub.publish(detection.ClassID)  # publish ClassID to ROS topic
		#rospy.loginfo('Published: ' + str(detection.ClassID))
	display.SetStatus("Object Detection | Network {:.0f} FPS".format(net.GetNetworkFPS()))