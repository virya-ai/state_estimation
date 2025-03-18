#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

ros::Publisher repub;
const double transform_x = 0;
const double transform_y = 0;
const double theta = 0.0; // Rotation angle in radians

void callback(const nav_msgs::Odometry::ConstPtr& msg) {
    geometry_msgs::PoseStamped repub_msg;

    // Extract original x and y coordinates
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    // Apply transformation
    double new_x = std::cos(theta) * x - std::sin(theta) * y + transform_x;
    double new_y = std::sin(theta) * x + std::cos(theta) * y + transform_y;

    // Populate the transformed pose message
    repub_msg.header = msg->header;  // Copy header information
    repub_msg.pose = msg->pose.pose; // Copy original pose information
    repub_msg.pose.position.x = new_x;
    repub_msg.pose.position.y = new_y;

    // Publish transformed pose
    repub.publish(repub_msg);
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "odom_to_ndt");
    ros::NodeHandle nh;

    // Create a publisher
    repub = nh.advertise<geometry_msgs::PoseStamped>("/ndt_pose1", 10);

    // Subscribe to the odometry topic
    ros::Subscriber sub = nh.subscribe("/odom", 10, callback);

    // Keep the node running
    ros::spin();
    return 0;
}
