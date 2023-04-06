#!/usr/bin/env python
from __future__ import print_function
from lib2to3.pytree import Node
import sys
import math
from tokenize import Double
import numpy as np
import time

from  numpy import array, dot
from quadprog import solve_qp
#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class GapBarrier:
    def __init__(self):
        #Topics & Subs, Pubs
        # Read paramters form params.yaml
        lidarscan_topic =rospy.get_param('~scan_topic')
        drive_topic = rospy.get_param('~nav_drive_topic')
        odom_topic=rospy.get_param('~odom_topic')


        self.max_steering_angle=rospy.get_param('~max_steering_angle')
        self.max_lidar_range=rospy.get_param('~scan_range')
        self.wheelbase=rospy.get_param('~wheelbase')
        self.CenterOffset=rospy.get_param('~CenterOffset')
        self.TrackWall=rospy.get_param('~TrackWall')
        self.k_p=rospy.get_param('~k_p')
        self.k_d=rospy.get_param('~k_d')
        self.angle_bl=rospy.get_param('~angle_bl')
        self.angle_al=rospy.get_param('~angle_al')
        self.angle_br=rospy.get_param('~angle_br')
        self.angle_ar=rospy.get_param('~angle_ar')
        self.n_pts_l=rospy.get_param('~n_pts_l')
        self.n_pts_r=rospy.get_param('~n_pts_r')
        self.velocity_high=rospy.get_param('~velocity_high')
        self.velocity_medium=rospy.get_param('~velocity_medium')
        self.velocity_low=rospy.get_param('~velocity_low')
        self.angle_threshold_high=rospy.get_param('~angle_threshold_high')
        self.angle_threshold_low=rospy.get_param('~angle_threshold_low')

        self.scan_beams=rospy.get_param('~scan_beams')
        self.safe_distance=rospy.get_param('~safe_distance')
        self.right_beam_angle=rospy.get_param('~right_beam_angle')
        self.left_beam_angle=rospy.get_param('~left_beam_angle')

        #Subscriptions,Publishers
        rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback,queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback,queue_size=1)

        self.marker_pub = rospy.Publisher("wall_markers", Marker, queue_size = 2)
        self.marker = Marker()

        self.drive_pub =rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)
        
        self.vel = 0

        self.ls_ang_inc=2*math.pi/self.scan_beams


        self.ls_str=int(round(self.scan_beams*self.right_beam_angle/(2*math.pi)))
        self.ls_end=int(round(self.scan_beams*self.left_beam_angle/(2*math.pi)))
        self.ls_len_mod=self.ls_end-self.ls_str+1
        self.ls_fov=self.ls_len_mod*self.ls_ang_inc
        self.angle_cen=self.ls_fov/2
        self.ls_len_mod2=0
        self.ls_data=[]


    # Pre-process LiDAR data    
    def preprocess_lidar(self, ranges):
        
        data=[]
     

        for i in range(self.ls_len_mod):
            if ranges[self.ls_str+i]<=self.safe_distance:
               data.append ([0,i*self.ls_ang_inc-self.angle_cen])
            elif ranges[self.ls_str+i]<=self.max_lidar_range:
                data.append ([ranges[self.ls_str+i],i*self.ls_ang_inc-self.angle_cen])
            else: 
                data.append ([self.max_lidar_range,i*self.ls_ang_inc-self.angle_cen])
        
        return np.array(data)



    # Return the start and end indices of the maximum gap in free_space_ranges
    def find_max_gap(self, proc_ranges):
        
        j=0
        str_indx=0;end_indx=0
        str_indx2=0;end_indx2=0
        
        range_sum = 0
        range_sum_new = 0

        for i in range (self.ls_len_mod):

            if proc_ranges[i,0]!=0:
                if j==0:
                    str_indx=i
                    range_sum_new = 0
                    j=1
                range_sum_new = range_sum_new + proc_ranges[i,0]
                end_indx=i
                
            if  j==1 and (proc_ranges[i,0]==0 or i==self.ls_len_mod-1):
               
                j=0
                #this if statemnet is used to idneitfy the longest sequence of cosective free space points
                #When a zero value is encountered, the length of the current sequence is compared to the length 
                # of the longest sequence seen so far, and the longer of the two is stored. The resulting length 
                # represents the maximum gap in the "free space" that can be used to plan safe trajectories.
                if  range_sum_new > range_sum: 
                        end_indx2= end_indx
                        str_indx2= str_indx
                        range_sum = range_sum_new 

        return str_indx2, end_indx2

    
    # start_i & end_i are the start and end indices of max-gap range, respectively
    # Returns index of best (furthest point) in ranges
    def find_best_point(self, start_i, end_i, proc_ranges):

        numerator = 0
        denom = 0
        for i in range(start_i, end_i + 1):
            #note that: proc_ranges[i,0] represents the range value (i.e., distance) of the i-th LIDAR return,
            #  while proc_ranges[i,1] represents the corresponding angle (heading) of that return 
            numerator = numerator + (proc_ranges[i,0] * proc_ranges[i,1])
            denom = denom + proc_ranges[i,0]
        best_heading = numerator / denom
        return best_heading

    def getWalls(self, left_obstacles, right_obstacles):

        P = np.array([[1,0], [0,1]])

        bl = np.full((self.n_pts_l), 1.0, dtype=np.float64)
        br = np.full((self.n_pts_r), 1.0, dtype=np.float64)

        a = np.array([0, 0])
        
        #constarints
        Cl= -(left_obstacles.T)
        Cr= -(right_obstacles.T)
        
        #astype = converts data into float
        wl = solve_qp(P.astype(np.float), a.astype(np.float), Cl.astype(np.float),  bl.astype(np.float), 0)[0]
        wr = solve_qp(P.astype(np.float), a.astype(np.float), Cr.astype(np.float),  br.astype(np.float), 0)[0]
        
        return  wl, wr 



    def lidar_callback(self, data):      

        ranges = data.ranges
       # self.scan_beams= len(data.ranges)- 1

        proc_ranges = self.preprocess_lidar(ranges)       

        str_indx,end_indx=self.find_max_gap(proc_ranges)           
        heading_angle =self.find_best_point(str_indx, end_indx, proc_ranges)
        

        index_l=int(round((self.angle_bl-self.angle_al)/(data.angle_increment*self.n_pts_l)))
        index_r=int(round((self.angle_ar-self.angle_br)/(data.angle_increment*self.n_pts_r)))

        mod_angle_al = self.angle_al + heading_angle

        if mod_angle_al > 2*math.pi:
            mod_angle_al = mod_angle_al - 2*math.pi
        elif mod_angle_al < 0:
            mod_angle_al = mod_angle_al + 2*math.pi
        
        mod_angle_br = self.angle_br + heading_angle

        if mod_angle_br > 2*math.pi:
            mod_angle_br = mod_angle_br - 2*math.pi
        elif mod_angle_br < 0:
            mod_angle_br = mod_angle_br + 2*math.pi
 

        start_indx_l=int(round(mod_angle_al/data.angle_increment))
        start_indx_r=int(round(mod_angle_br/data.angle_increment))

        obstacle_points_l=np.zeros((self.n_pts_l,2))
        obstacle_points_r=np.zeros((self.n_pts_r,2))


        for k in range(0, self.n_pts_l):

            obs_index = (start_indx_l+k*index_l) % self.scan_beams
            obs_range= data.ranges[obs_index]
            if obs_range >=self.max_lidar_range:
                obs_range = self.max_lidar_range
                
                
            obstacle_points_l[k][0]= - obs_range*math.cos(mod_angle_al+k*index_l*data.angle_increment)
            obstacle_points_l[k][1]= - obs_range*math.sin(mod_angle_al+k*index_l*data.angle_increment)

        for k in range(0,self.n_pts_r):
            
            obs_index = (start_indx_r+k*index_r) % self.scan_beams
            obs_range= data.ranges[obs_index]
            if obs_range >=self.max_lidar_range:
                obs_range = self.max_lidar_range
                
            obstacle_points_r[k][0]= - obs_range*math.cos(mod_angle_br+k*index_r*data.angle_increment)
            obstacle_points_r[k][1]= - obs_range*math.sin(mod_angle_br+k*index_r*data.angle_increment)
        

        wl, wr = self.getWalls(obstacle_points_l, obstacle_points_r) 

        dl = 1/math.sqrt(np.dot(wl.T,wl))
        dr = 1/math.sqrt(np.dot(wr.T,wr))

        wl_h = wl*dl
        wr_h = wr*dr

        self.marker.header.frame_id = "base_link"
        self.marker.header.stamp = rospy.Time.now() 
        self.marker.type = Marker.LINE_LIST
        self.marker.id = 0
        self.marker.action= Marker.ADD
        self.marker.scale.x = 0.1
        self.marker.color.a = 1.0
        self.marker.color.r = 0.5
        self.marker.color.g = 0.5
        self.marker.color.b = 0.0
        self.marker.pose.orientation.w = 1

        self.marker.lifetime=rospy.Duration(0.1)
    

        self.marker.points = []
        
        line_len = 1
        self.marker.points.append(Point(dl*(-wl_h[0]-line_len*wl_h[1]), dl*(-wl_h[1]+line_len*wl_h[0]) , 0))
        self.marker.points.append(Point(dl*(-wl_h[0]+line_len*wl_h[1]), dl*(-wl_h[1]-line_len*wl_h[0]) , 0))
        self.marker.points.append(Point(dr*(-wr_h[0]-line_len*wr_h[1]), dr*(-wr_h[1]+line_len*wr_h[0]) , 0))
        self.marker.points.append(Point(dr*(-wr_h[0]+line_len*wr_h[1]), dr*(-wr_h[1]-line_len*wr_h[0]) , 0))
        self.marker.points.append(Point(0, 0 , 0))
        self.marker.points.append(Point(line_len*math.cos(heading_angle), line_len*math.sin(heading_angle), 0))
   
        self.marker_pub.publish(self.marker)

        
        if self.vel >= 0.01 or self.vel <= -0.01:

            d_tilde= dl-dr - self.CenterOffset
            d_tilde_dot=self.vel*(wl_h[0]-wr_h[0])
            delta_d = math.atan((self.wheelbase*(self.k_p*d_tilde+self.k_d*d_tilde_dot))/((self.vel**2)*(-wl_h[1]+wr_h[1])))
        
        else:
            delta_d = 0
            

        if delta_d >=self.max_steering_angle:
            delta_d=self.max_steering_angle
        elif delta_d<=-self.max_steering_angle:
            delta_d =-self.max_steering_angle
        
        angle_deg=abs(delta_d)*180/math.pi
        
        if angle_deg>=0 and angle_deg<=self.angle_threshold_low:
            velocity=self.velocity_high
        elif angle_deg > self.angle_threshold_low and angle_deg <= self.angle_threshold_high:
            velocity=self.velocity_medium
        else:
            velocity=self.velocity_low

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
    rospy.init_node("GapWallFollow_node", anonymous=True)
    wf = GapBarrier()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
