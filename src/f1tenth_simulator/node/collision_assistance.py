#!/usr/bin/env python
from __future__ import print_function
from cmath import tan
import sys
import math
import numpy as np

# ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan, Joy
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry


class CollisionAssistance:
    def __init__(self):
        # Read paramters form params.yaml
        lidarscan_topic = rospy.get_param('~scan_topic')
        drive_topic = rospy.get_param('~collision_assistance_drive_topic')
        odom_topic = rospy.get_param('~odom_topic')
        joy_topic = rospy.get_param('~joy_topic')
        self.max_lidar_range = rospy.get_param('~scan_range')
        self.joy_speed_axis = rospy.get_param('~joy_speed_axis')
        self.joy_angle_axis = rospy.get_param('~joy_angle_axis')
        self.max_steering_angle = rospy.get_param('~max_steering_angle')
        self.max_speed = rospy.get_param('~max_speed')
        self.scan_beams = rospy.get_param('~scan_beams')
        self.wheelbase = rospy.get_param('~wheelbase')
        self.d_obs = rospy.get_param("~distance_to_obstacle_th")
        self.f_gain = rospy.get_param("~force_gain")
        self.etha_vel = rospy.get_param("~velocity_correction_gain")
        self.etha_delta = rospy.get_param("~steering_correction_gain")

        # Subscriptions,Publishers
        rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback, queue_size=1)
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber(joy_topic, Joy, self.Joy_callback, queue_size=1)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

        # Initilize Joystick velcocity and steering commands
        self.vel_joy = 0
        self.steer_joy = 0

        # Lidar parameters

        # compute beam angle increment
        self.ls_ang_inc = 2 * math.pi / self.scan_beams

        # start index for 180 deg. FOV
        self.ls_str = int(round(self.scan_beams / 4))
        # number of laser beams in the front 180 deg. FOV
        self.ls_len_mod = int(round(self.scan_beams / 2))
        # front center beam angle
        self.angle_cen = self.ls_len_mod * self.ls_ang_inc / 2
        # number of laser returns falling inside maximum valid range
        self.ls_len_mod2 = 0
        self.ls_data = []

        # initialize vehicle velocity
        self.vel = 0
        # initialize collision avoidance velocity command
        self.vel_x = 0
        # initialize number of detected obstacles
        self.obs_indx = np.zeros((self.scan_beams, 2), dtype=int)

    # Preprocess the LiDAR scan array.
    def preprocess_lidar(self, ranges):

        data = []
        j = 0
        for i in range(self.ls_len_mod):
            if ranges[self.ls_str + i] <= self.max_lidar_range:
                data.append([ranges[self.ls_str + i], i * self.ls_ang_inc - self.angle_cen])
                j = j + 1
        self.ls_len_mod2 = j
        return np.array(data)

    # Find unsafe obstacles and return number of unsafe obstacles
    #  and the closest point of each unsafe obstacle to the vehicle
    def obst_idnt(self, ls_ranges):

        indx = np.zeros((2), dtype=int)
        j = 0
        nm_obs = 0

        for i in range(self.ls_len_mod2):
            if ls_ranges[i, 0] <= self.d_obs and i < self.ls_len_mod2 - 1:
                if j == 0:
                    indx[0] = i
                    j = 1
                indx[1] = i
            else:
                j = 0
                if indx[1] - indx[0] > 0:
                    self.obs_indx[nm_obs, :] = indx
                    nm_obs = nm_obs + 1
                    indx[1] = 0
                    indx[0] = 0
        #arg_ls_obs is an array that contains the LiDAR measurement indices corresponding to
        # the nearest point on each identified potential obstacle.
        arg_ls_obs = np.zeros(nm_obs, dtype=int)

        for i in range(nm_obs):
            # find index of minimum range in obstacle range
            #arg_ls_obs[i] = np.argmin(..., 0])
            arr = ls_ranges[self.obs_indx[i, 0]:self.obs_indx[i, 1] + 1]
            arg_ls_obs[i] = np.argmin(arr, 0)
            #add index of minimum range to first index of obstacle range
            arg_ls_obs[i] = self.obs_indx[i, 0] + arg_ls_obs[i]

        return nm_obs, arg_ls_obs

    # compute corrective force acting on the vehicle
    def ptn_fld(self, nm_obs, arg_ls_obs, ls_ranges):

        f_rep_x = 0
        f_rep_y = 0

        for i in range(nm_obs):
            d = ls_ranges[arg_ls_obs[i], 0]
            theta = ls_ranges[arg_ls_obs[i], 1]
            #self.f_gain is just the force gain in the params file
            f_rep_x = f_rep_x + self.f_gain * self.etha_vel
            f_rep_y = f_rep_y + self.f_gain * self.etha_vel

        return f_rep_x, f_rep_y

    def odom_callback(self, odom_msg):
        # update current speed
        self.vel = odom_msg.twist.twist.linear.x

    def Joy_callback(self, joy_msg):
        # update Joy
        self.vel_joy = self.max_speed * joy_msg.axes[self.joy_speed_axis]
        self.steer_joy = self.max_steering_angle * joy_msg.axes[self.joy_angle_axis]

    def lidar_callback(self, data):
        ranges = data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        nm_obs, arg_ls_obs = self.obst_idnt(proc_ranges)

        # Calculate the potential field forces
        f_tot_x, f_tot_y = self.ptn_fld(nm_obs, arg_ls_obs, proc_ranges)

        # Calculate desired velocity and steering angle
        self.vel_x = self.vel + f_tot_x

        #i think the vel_x is for the velocity and d is for idk
        vel_d = ...

        if self.vel_joy >= 0 and vel_d < 0:
            vel_d = 0

        u0 = self.etha_delta * math.tan(self.steer_joy) + (1-self.etha_delta)* (self.wheelbase/max(pow(self.vel,2),pow(10,-4)))*f_tot_y
        delta_d = math.atan((u0 - math.tan(self.steer_joy)) / (1 + u0 * math.tan(self.steer_joy))) + self.steer_joy

        if delta_d >= self.max_steering_angle:
            delta_d = self.max_steering_angle
        elif delta_d <= -self.max_steering_angle:
            delta_d = -self.max_steering_angle
        if vel_d >= self.max_speed:
            vel_d = self.max_speed
        elif vel_d <= -self.max_speed:
            vel_d = -self.max_speed

        # Publish Ackermann commands
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.steering_angle = delta_d
        drive_msg.drive.speed = vel_d
        self.drive_pub.publish(drive_msg)


def main(args):
    rospy.init_node("collision_assitance", anonymous=True)
    rfgs = CollisionAssistance()
    rospy.sleep(.1)
    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
