

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <vesc_msgs/VescStateStamped.h>
#include <geometry_msgs/TransformStamped.h>


#include <cmath>
#include <iostream>


class RacecarExperiment {
private:
    // A ROS node
    ros::NodeHandle n;

    // Listen to drive commands
    ros::Subscriber drive_sub;

    // Listen to vesc
    ros::Subscriber vesc_state_sub;
    ros::Subscriber servo_sub;

    // Listen to lidar
    ros::Subscriber lidar_sub;

    // Listen to IMU
     ros::Subscriber imu_sub;

    // Listen to mux messages
    ros::Subscriber mux_sub;

    // Publish lidar and odometry 
    ros::Publisher lidar_pub;
    ros::Publisher odom_pub;
    ros::Publisher odom_imu_pub;
    

    // Publish drive data to vesc
    ros::Publisher erpm_pub;
    ros::Publisher servo_pub;
    ros::Publisher brake_pub;

    // Transform broadcaster for odometry 
    tf::TransformBroadcaster odom_broadcaster, odom_imu_broadcaster;
    geometry_msgs::TransformStamped odom_transform, odom_imu_transform;
    std::string odom_frame, base_frame;

    // The car state and parameters
    double desired_speed=0;double desired_steer_ang=0; double last_servo=0; double last_rpm=0;

    
    std_msgs::Float64 erpm_msg;
    std_msgs::Float64 servo_msg;
    std_msgs::Float64 brake_msg;

    std_msgs::Float64 last_servo_state; // Last recived servo state
    vesc_msgs::VescStateStamped last_vesc_state; // Last received vesc state

    // publish odometry message
    nav_msgs::Odometry odom, odom_imu;

    bool    waiting_on_init_yaw = true;
    double  init_yaw=0; double imu_yaw; double omega=0;

    // Odometry parameters
    double speed_to_erpm_gain, speed_to_erpm_offset,
        steering_angle_to_servo_gain, steering_angle_to_servo_offset, wheelbase;
    double yaw=0.0;double x_odom=0.0;double y_odom=0.0;
    double yaw_imu =0; double x_odom_imu=0; double y_odom_imu=0;

    double max_acceleration, max_servo_speed, driver_smoother_rate, max_delta_servo,max_delta_rpm;
    
    ros::Timer update_command;

    // Mux controller array
    std::vector<bool> mux_controller;
    int mux_size;

public:
RacecarExperiment() {
    // Initialize the node handle
    n = ros::NodeHandle("~");
    
    // Get topic names
    std::string drive_topic, scan_topic, odom_topic, mux_topic, imu_topic;
    n.getParam("drive_topic", drive_topic);
    n.getParam("scan_topic", scan_topic);
    n.getParam("odom_topic", odom_topic);
    n.getParam("mux_topic", mux_topic);
    n.getParam("imu_topic", imu_topic);

    n.getParam("mux_size", mux_size);
    
    //Get vesc gains
    n.getParam("speed_to_erpm_gain", speed_to_erpm_gain);
    n.getParam("speed_to_erpm_offset", speed_to_erpm_offset);
    n.getParam("steering_angle_to_servo_gain", steering_angle_to_servo_gain);
    n.getParam("steering_angle_to_servo_offset", steering_angle_to_servo_offset);
    desired_steer_ang=steering_angle_to_servo_offset;
    last_servo=steering_angle_to_servo_offset;
    last_servo_state.data=steering_angle_to_servo_offset;

    // Get car parameters
    n.getParam("wheelbase", wheelbase);
    n.getParam("max_accel", max_acceleration);
    n.getParam("max_steering_vel",max_servo_speed);
    n.getParam("driver_smoother_rate",driver_smoother_rate);
    
 
    // Get odometry and base frame names
    n.getParam("odom_frame", odom_frame);
    n.getParam("base_frame", base_frame);



 // uncomment if you would like to publish odomotery "transformatons" over ROS
 // Note that there is one transformation for wheel-odometry using VESC only and a second one that use IMU yaw measurement 

    // start transform braodcaster for  wheel odometry (first part)
    odom_transform.header.frame_id = odom_frame;
    odom_transform.child_frame_id = base_frame;

    // start transform braodcaster for odometry with imu data (second part)
    odom_imu_transform.header.frame_id = odom_frame;
    odom_imu_transform.child_frame_id = "base_link_imu";


    // Make a publisher for laser scan messages
    lidar_pub = n.advertise<sensor_msgs::LaserScan>(scan_topic, 1);

    // Make a publisher for odometry messages (first part)
    odom_pub = n.advertise<nav_msgs::Odometry>(odom_topic, 1);
    odom.header.frame_id = base_frame;
    odom.child_frame_id = odom_frame;

// Make a publisher for odometry messages with imu data (second part)
    odom_imu_pub = n.advertise<nav_msgs::Odometry>("/odom_imu", 1);
    odom_imu.header.frame_id = base_frame;
    odom_imu.child_frame_id = "odom_imu";
    
    // Make a publisher for drive message
    erpm_pub = n.advertise<std_msgs::Float64>("/commands/motor/speed", 1);
    servo_pub = n.advertise<std_msgs::Float64>("/commands/servo/position", 1);
    brake_pub = n.advertise<std_msgs::Float64>("/commands/motor/brake", 1);


    // Start a subscriber to listen to drive commands
    drive_sub = n.subscribe(drive_topic, 1, &RacecarExperiment::driver_callback, this);

    // Start a subscriber to vesc
    vesc_state_sub= n.subscribe("/sensors/core", 1, &RacecarExperiment::vesc_callback, this);
    servo_sub= n.subscribe("/sensors/servo_position_command", 1,&RacecarExperiment::servo_callback, this);
  
   // Start a subsriber to lidar
    lidar_sub = n.subscribe("/scan2", 1, &RacecarExperiment::laser_callback, this);
 
   // Star a subscriber to IMU
    imu_sub= n.subscribe(imu_topic, 1, &RacecarExperiment::imu_callback, this);

   // Start a subscriber to listen to mux messages
    mux_sub = n.subscribe(mux_topic, 1, &RacecarExperiment::mux_callback, this);

    
    max_delta_servo = std::abs(steering_angle_to_servo_gain * max_servo_speed / driver_smoother_rate);
    max_delta_rpm = std::abs(speed_to_erpm_gain * max_acceleration / driver_smoother_rate);
    update_command = n.createTimer(ros::Duration(1.0/driver_smoother_rate), &RacecarExperiment::publish_driver_command, this);
    

    // initialize mux controller
    mux_controller.reserve(mux_size);
    for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = false;
        }


    ROS_INFO("Experiment constructed.");

}

    void driver_callback(const ackermann_msgs::AckermannDriveStamped & msg) {
        desired_speed = msg.drive.speed;
        desired_steer_ang = msg.drive.steering_angle;
        desired_speed=speed_to_erpm_gain * desired_speed + speed_to_erpm_offset;
        desired_steer_ang = steering_angle_to_servo_gain * desired_steer_ang + steering_angle_to_servo_offset;
    }

    void publish_driver_command( const ros::TimerEvent&){
        double desired_delta = desired_steer_ang-last_servo;
        double clipped_delta = std::max(std::min(desired_delta, max_delta_servo), -max_delta_servo);
        double smoothed_servo = last_servo + clipped_delta;
        last_servo = smoothed_servo;
        servo_msg.data=smoothed_servo;

        double desired_rpm = desired_speed-last_rpm;
        double clipped_rpm = std::max(std::min(desired_rpm, max_delta_rpm), -max_delta_rpm);
        double smoothed_rpm = last_rpm + clipped_rpm;
        last_rpm = smoothed_rpm; 
        erpm_msg.data=smoothed_rpm;     

        servo_pub.publish(servo_msg);
        erpm_pub.publish(erpm_msg);

    }

    void servo_callback(const std_msgs::Float64 & servo){
        last_servo_state= servo;
    }
      

    void vesc_callback(const vesc_msgs::VescStateStamped & state){
        double current_speed = ( state.state.speed - speed_to_erpm_offset ) / speed_to_erpm_gain;

        if (std::fabs(current_speed) < 0.15 || std::fabs(state.state.current_motor)<0.01 ){
            current_speed = 0.0;
        }

        double current_steering_angle, current_angular_velocity;
        current_steering_angle =
            ( last_servo_state.data - steering_angle_to_servo_offset) / steering_angle_to_servo_gain;
        current_angular_velocity = current_speed * tan(current_steering_angle) / wheelbase;


        // calc elapsed time
        ros::Duration dt = state.header.stamp - last_vesc_state.header.stamp;

       // save state for next time
        last_vesc_state = state;



// following lines must be completed

        // odometry information
        double x_dot = current_speed * cos(yaw);
        double y_dot = current_speed * sin(yaw);
        x_odom+= x_dot * dt.toSec();
        y_odom+= y_dot * dt.toSec();
        yaw+= current_angular_velocity * dt.toSec();

        
  
        // Publish odometry message
        odom.header.stamp = state.header.stamp;
    
        // Position
        odom.pose.pose.position.x = x_odom;
        odom.pose.pose.position.y = y_odom;
        odom.pose.pose.orientation.x = 0.0;
        odom.pose.pose.orientation.y = 0.0;

        // following lines must be completed
        odom.pose.pose.orientation.z = sin(yaw/2);
        odom.pose.pose.orientation.w = cos(yaw/2);

        // Velocity
        odom.twist.twist.linear.x = current_speed;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.linear.z = 0;
      
        odom.twist.twist.angular.z = current_angular_velocity;
        
        odom_pub.publish(odom);

        // Publish odometry tranformation
	// note that this line of code is just setting the variables of odom_transform message to be published
        odom_transform.transform.translation.x= x_odom;
        odom_transform.transform.translation.y= y_odom;
        odom_transform.transform.translation.z= 0;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);
        odom_transform.transform.rotation = odom_quat;

        odom_transform.header.stamp=state.header.stamp;

   // uncomment following line to publish "odom" tranformation 

        //odom_broadcaster.sendTransform(odom_transform);


        
       // odometry using imu heading measurement
       
     // uncomment if publishing modified odometry using heading angle measurement from IMU

   
       if (!waiting_on_init_yaw)
       {
        double yaw_imu =imu_yaw-init_yaw;
        x_odom_imu += current_speed * cos(yaw_imu) * dt.toSec();
        y_odom_imu += current_speed * sin(yaw_imu) * dt.toSec();

        odom_imu.pose.pose.position.x = x_odom_imu;
        odom_imu.pose.pose.position.y = y_odom_imu;
        odom_imu.pose.pose.position.z = 0;

        odom_imu.pose.pose.orientation.x = 0.0;
        odom_imu.pose.pose.orientation.y = 0.0;
        odom_imu.pose.pose.orientation.z = sin(yaw_imu/2);
        odom_imu.pose.pose.orientation.w = cos(yaw_imu/2);

        odom_imu.twist.twist.linear.x = current_speed;
        odom_imu.twist.twist.linear.y = 0.0;
        odom_imu.twist.twist.linear.z = 0;
        
        odom_imu.twist.twist.angular.z = omega;
        odom_imu_pub.publish(odom_imu);

      // Publish odometry tranformation
        odom_imu_transform.transform.translation.x= x_odom_imu;
        odom_imu_transform.transform.translation.y= y_odom_imu;
        odom_imu_transform.transform.translation.z= 0;
        geometry_msgs::Quaternion odom_imu_quat = tf::createQuaternionMsgFromYaw(yaw_imu);
        odom_imu_transform.transform.rotation = odom_imu_quat;

        odom_imu_transform.header.stamp=state.header.stamp;
        
        odom_imu_broadcaster.sendTransform(odom_imu_transform);


      }




}
  

    void laser_callback(const sensor_msgs::LaserScan & msg) {

       // re-arrange lidar data

        int n1=msg.ranges.size()/2;
        std::vector<float> scan_ranges=msg.ranges;
        std::vector<float> scan_intensities=msg.intensities;
        for (int i=0;i<n1;i++){
            scan_ranges[i]=msg.ranges[n1+i];
            scan_ranges[n1+i]=msg.ranges[i];
            scan_intensities[i]=msg.intensities[n1+i];
            scan_intensities[n1+i]=msg.intensities[i];
        }

       // Publish lidar information
        sensor_msgs::LaserScan scan_msg;
        scan_msg.header.stamp = msg.header.stamp;
        scan_msg.header.frame_id = msg.header.frame_id;
        scan_msg.angle_min = msg.angle_min+3.141592;
        scan_msg.angle_max =  msg.angle_max+3.141592;
        scan_msg.angle_increment = msg.angle_increment;
        scan_msg.range_max = msg.range_max;    
        scan_msg.ranges = scan_ranges;
        scan_msg.intensities = scan_intensities;
        lidar_pub.publish(scan_msg);
    }
 


	void imu_callback(const sensor_msgs::Imu& imu_msg)
	{
  
   	tf::Quaternion quat;

    	tf::quaternionMsgToTF(imu_msg.orientation, quat);
        tfScalar yaw, pitch, roll;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        if (waiting_on_init_yaw)
          {
            init_yaw = yaw;
            waiting_on_init_yaw = false;
          }
   
        imu_yaw = yaw;
        omega = imu_msg.angular_velocity.z;

	}


    void mux_callback(const std_msgs::Int32MultiArray & msg) {
        // Publish zero command when emergency brake is activeted
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = bool(msg.data[i]);
        }
        if (mux_controller[3]==true){
            // brake_msg.data=0;
            // brake_pub.publish(brake_msg);
            erpm_msg.data=0.0;
            desired_speed=0.0;
            erpm_pub.publish(erpm_msg);
            }
        }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "racecar_experiment");
    RacecarExperiment rs;
    ros::spin();
    return 0;
}
