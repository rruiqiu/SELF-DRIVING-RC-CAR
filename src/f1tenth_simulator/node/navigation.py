#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import time

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry




class WallFollow:
    def __init__(self):
        #Topics & Subs, Pubs
        # Read paramters form params.yaml
        lidarscan_topic =rospy.get_param('~scan_topic')
        drive_topic = rospy.get_param('~nav_drive_topic')
        odom_topic=rospy.get_param('~odom_topic')

        self.t_prev=rospy.get_time()
        self.max_steering_angle=rospy.get_param('~max_steering_angle')
        self.max_lidar_range=rospy.get_param('~scan_range')
        self.wheelbase=rospy.get_param('~wheelbase')
        self.CenterOffset=rospy.get_param('~CenterOffset')
        self.DistanceLeft=rospy.get_param('~DistanceLeft')
        self.DistanceRight=rospy.get_param('~DistanceRight')
        self.TrackWall=rospy.get_param('~TrackWall')
        self.k_p=rospy.get_param('~k_p')
        self.k_d=rospy.get_param('~k_d')
        self.angle_bl=rospy.get_param('~angle_bl')
        self.angle_al=rospy.get_param('~angle_al')
        self.angle_br=rospy.get_param('~angle_br')
        self.angle_ar=rospy.get_param('~angle_ar')
        self.velocity_high=rospy.get_param('~velocity_high')
        self.velocity_medium=rospy.get_param('~velocity_medium')
        self.velocity_low=rospy.get_param('~velocity_low')
        self.angle_threshold_high=rospy.get_param('~angle_threshold_high')
        self.angle_threshold_low=rospy.get_param('~angle_threshold_low')

    
        #Subscriptions,Publishers
        rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback,queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback,queue_size=1)

        self.drive_pub =rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        
        self.vel = 0
        #formulas in page 15
        self.thetal= self.angle_bl - self.angle_al
        self.thetar= self.angle_ar - self.angle_br


    def getRange(self, data, angle):
        
        index=int(round(angle/data.angle_increment))
        data2=data.ranges[index]

        if data2>=self.max_lidar_range:
            data2=self.max_lidar_range
        return data2



    def lidar_callback(self, data):      

        dis_lsr_al=self.getRange(data, self.angle_al)
        dis_lsr_bl=self.getRange(data, self.angle_bl)

        dis_lsr_ar=self.getRange(data, self.angle_ar)
        dis_lsr_br=self.getRange(data, self.angle_br)

        betar=math.atan((dis_lsr_ar*math.cos(self.thetar)-dis_lsr_br)/(dis_lsr_ar*math.sin(self.thetar)))
        betal=math.atan((dis_lsr_al*math.cos(self.thetal)-dis_lsr_bl)/(dis_lsr_al*math.sin(self.thetal)))

        alphal = -1*betal - self.angle_bl + math.pi*(3/2)
        # 3pi/2 = 270 deg.
        

        alphar = betar - self.angle_br + math.pi/2

        # distance to the left walls
        dl=dis_lsr_bl*math.cos(betal)
        # distance to the right walls
        dr=dis_lsr_br*math.cos(betar)


        
        if self.vel >= 0.01 or self.vel <= -0.01:
            
            # Track left wall 
            if self.TrackWall == 1:
                d_tilde = self.DistanceLeft-dl
                d_dot= -self.vel * math.sin(alphal)
                #delta_d = math.atan(-(self.wheelbase)/((self.vel**2)*math.cos(alphal))*(-self.k_p*d_tilde-self.k_d*d_dot))
                delta_d = math.atan(-(self.wheelbase*(self.k_p*d_tilde - self.k_d*d_dot))/((self.vel**2)*math.cos(alphal)))

            # Trackl right wall 
            elif self.TrackWall == 2:
                d_tilde = self.DistanceRight-dr
                d_dot= self.vel*math.sin(alphar)
                #delta_d = math.atan((self.wheelbase)/((self.vel**2)*math.cos(alphar))*(-self.k_p*d_tilde-self.k_d*d_dot))
                delta_d = math.atan((self.wheelbase*(self.k_p*d_tilde - self.k_d*d_dot))/((self.vel**2)*math.cos(alphar)))

            # Track both walls
            else :
                d_tilde= dl -dr -self.CenterOffset
                d_tilde_dot=-self.vel*math.sin(alphal)-self.vel*math.sin(alphar)
                delta_d = math.atan((self.wheelbase*(self.k_p*d_tilde+self.k_d*d_tilde_dot))/((self.vel**2)*(math.cos(alphal)+math.cos(alphar))))
        
        else:
            

            delta_d = 0
            
        # if the steering angle exceeds the threshold value.
        if delta_d >=self.max_steering_angle:
            delta_d=self.max_steering_angle
        elif delta_d<=-self.max_steering_angle:
            delta_d =-self.max_steering_angle
        
        #radians to degree conversion
        angle_deg=abs(delta_d)*180/math.pi
        #angle_threshold_low: 10, defined in params.yaml file
        #angle_threshold_high: 20
        # velocity_high: 1.5
        # velocity_medium: 1.0
        # velocity_low: 0.5
        if angle_deg>=0 and angle_deg<=self.angle_threshold_low:
            velocity=self.velocity_high
        elif angle_deg > self.angle_threshold_low and angle_deg <= self.angle_threshold_high:
            velocity=self.velocity_medium
        else:
            velocity=self.velocity_low

        #if the stering angle is between zero and threshold_low, the velocity will be high
        #if between low and high, the v will be medium
        #if exceeds the high angle, the v will be low
        #to make sure the aev will not speed too fast when it's not driving horizontally.

        # Publish to driver topic
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.steering_angle = delta_d
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)



    def odom_callback(self, odom_msg):
        # update current speed
        self.vel = odom_msg.twist.twist.linear.x


def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)