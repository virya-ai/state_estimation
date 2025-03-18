#include <ros/ros.h>
#include <std_msgs/Float64.h>  // ROS 1 message type for Float64
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>   // Include for tf functions like createQuaternionMsgFromYaw
#include <nav_msgs/Odometry.h>        // Include for Odometry message
#include <nav_msgs/Path.h>
#include <dead_reckoning/BicycleModelSimple.hpp>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <fstream>
#include <eigen3/Eigen/Dense>

// Bicycle Odometry class
class BicycleOdometry {
public:
    // Constructor
    
    BicycleOdometry(ros::NodeHandle& nh) :
    bicycle_model(std::vector<double>{0.0, 0.86}) {
        
        downsample_rate = 5; 
        index = 0; 
        odom_pub = nh.advertise<nav_msgs::Odometry>("/bicycle/odom", 10);
        path_pub = nh.advertise<nav_msgs::Path>("/kinematic_model/path", 10);
        steer_sub = nh.subscribe("/currentSteerAngle", 10, &BicycleOdometry::steerCallback, this);
        speed_sub = nh.subscribe("/vehicle_speed", 10, &BicycleOdometry::speedCallback, this);
        odom_steer_angle_pub = nh.advertise<std_msgs::Float64>("/bicycle/orientation", 10);
        poses_ = {};

        // Getting the folder number 
        std::ifstream run_no_file("/home/catkin_ws/src/data_processing/data/online_plotting/run_no.txt");
        if(!run_no_file.is_open()) throw std::runtime_error("Could not open file");
        std::string line;
        if (run_no_file.good()) {
            std::getline(run_no_file, line);
            folder_no_ = std::stoi(line);         
        }
        run_no_file.close();

        std::stringstream file_path_stream;
        file_path_stream << "/home/catkin_ws/src/data_processing/data/online_plotting/run_no" 
                        << folder_no_<<"/encoder_odom.csv";

        
        file_path_ = file_path_stream.str();
        std::cout << file_path_ << std::endl;
        output_file.open(file_path_);
        output_file << "timestamp, x , y \n"; 

        x = 0.0;
        y = 0.0;
        theta = 0.0;
        currentSteerAngle = 0.0;
        vehicleSpeed = 0.0;
    }

    void steerCallback(const std_msgs::Float64::ConstPtr& msg) {
        currentSteerAngle = (msg->data - 6.4) * M_PI /180;
    }

    void speedCallback(const std_msgs::Float64::ConstPtr& msg) {
        // vehicleSpeed = msg->data * 0.277778 * 4 / 5.8 * 21.82/22.8;
        // vehicleSpeed = msg->data * 0.277778 * 1.27;
        vehicleSpeed = msg->data * 0.35;
    }

    void computeOdometry() {
        double dt = 0.05;

        bicycle_model.update( vehicleSpeed,  currentSteerAngle, dt);
        auto odometry = bicycle_model.get_state();
        x = odometry[0];  
        y = odometry[1];  
        theta = odometry[2];

        if (theta > M_PI) {
            theta -= 2 * M_PI;
        } else if (theta < -M_PI) {
            theta += 2 * M_PI;
        }

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromYaw(theta);
        pose.pose.orientation = quaternion; 
        pose.header.frame_id = "camera_init";

        std_msgs::Float64 orienation_msg;
        orienation_msg.data= theta *180/M_PI; // Degrees 
        odom_steer_angle_pub.publish(orienation_msg);

        if (index % downsample_rate == 0){poses_.push_back(pose);}
        index++; 

        publishOdometry();
    }


    void publishPath(){
        auto path_msg = nav_msgs::Path();
        path_msg.poses = poses_;
        path_msg.header.frame_id = "camera_init";
        path_pub.publish(path_msg);
    }

    void logOdometry(){
        auto time = ros::Time::now();
    }
        

private:
    // ROS Publisher and Subscribers
    ros::Publisher odom_pub;
    ros::Publisher path_pub;
    ros::Publisher odom_steer_angle_pub;
    ros::Subscriber steer_sub;
    ros::Subscriber speed_sub;
    int folder_no_; 
    int downsample_rate;
    int index;
    std::string file_path_;
    std::ofstream output_file;

    std::vector<geometry_msgs::PoseStamped> poses_; 
    BicycleModelSimple bicycle_model;

    // Odometry variables
    double x, y, theta;
    double currentSteerAngle, vehicleSpeed;

    // Function to publish the odometry
    void publishOdometry() {
        // Create the Odometry message
        nav_msgs::Odometry odom_msg;

        // Set the timestamp and frame
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "odom";

        // Set the position (x, y, z)
        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;

        // Set the orientation as a quaternion
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
        odom_msg.pose.pose.orientation = odom_quat;

        // Set the linear velocity
        odom_msg.twist.twist.linear.x = vehicleSpeed;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;

        // Set the angular velocity (yaw rate)
        odom_msg.twist.twist.angular.z = vehicleSpeed /0.8 * tan(currentSteerAngle);  // Simple kinematic model for angular velocity

        // Publish the odometry message
        odom_pub.publish(odom_msg);
    }
};

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "bicycle_odometry_node");
    ros::NodeHandle nh;

    // Create the BicycleOdometry object
    BicycleOdometry bicycle_odometry(nh);

    // Run the ROS loop
    ros::Rate rate(20);  // 10 Hz loop for odometry calculation
    while (ros::ok()) {
        // Compute the odometry
        bicycle_odometry.computeOdometry();
        bicycle_odometry.publishPath();
        bicycle_odometry.logOdometry();

        // Spin once to handle callbacks
        ros::spinOnce();

        // Sleep to maintain the desired rate
        rate.sleep();
    }

    return 0;
}
