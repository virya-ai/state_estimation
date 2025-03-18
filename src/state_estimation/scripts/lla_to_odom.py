#!/usr/bin/env python2
import rospy
import math
import numpy as np
import tf2_ros
import geometry_msgs.msg
from collections import deque
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import NavSatFix, Imu
from utils import GPS_utils

# Global Variables
gnss_pub = None
gps_converter = GPS_utils()
gnss_odom_msg = Odometry()
fix_status = 0
imu_quat = [0, 0, 0, 1]  # Default quaternion from IMU

# Set the ENU origin
lat = 12.818808815442026
long = 77.69415522995406
alt = 815.1511848832016
gps_converter.setENUorigin(lat, long, alt)

# TF Broadcaster
tf_broadcaster = None

# Store past coordinates
x_coords = deque(maxlen=5)
y_coords = deque(maxlen=5)

# ROS Publishers
reset_pub = None  # Will be initialized in listener()
heading_quat = [0, 0, 0, 1]  # Default quaternion from localization

# def imu_callback(data):
#     """Callback function to update orientation from IMU quaternion."""
#     global imu_quat
#     imu_quat = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]


def localization_callback(data):
    """Callback function to update odometry from localization."""
    # Will only extract heading info from this message
    global heading_quat
    orientation = data.pose.pose.orientation
    heading_quat = [orientation.x, orientation.y, orientation.z, orientation.w]
    print("Heading callback: ", heading_quat)

def gnss_callback(data):
    """Callback function for GNSS data to convert GPS to ENU and publish odometry + TF."""
    global gnss_odom_msg, fix_status, x_coords, y_coords, reset_pub, imu_quat, tf_broadcaster, heading_quat
    global gnss_pub
    # Convert GPS to ENU
    lat, lon, alt = data.latitude, data.longitude, data.altitude
    fix_status = data.status.status
    coords = gps_converter.geo2enu(lat, lon, alt)

    # Store coordinates
    x_coords.append(coords[0].item())
    y_coords.append(coords[1].item())

    # Update odometry message
    gnss_odom_msg = Odometry()
    gnss_odom_msg.header.stamp = rospy.Time.now()
    gnss_odom_msg.header.frame_id = "odom"
    # gnss_odom_msg.child_frame_id = "base_link"
    gnss_odom_msg.pose.pose.position.x = coords[0].item()
    gnss_odom_msg.pose.pose.position.y = coords[1].item()
    gnss_odom_msg.pose.pose.position.z = 0
    print("Orientation: ", heading_quat)
    gnss_odom_msg.pose.pose.orientation.x = heading_quat[0]
    gnss_odom_msg.pose.pose.orientation.y = heading_quat[1]
    gnss_odom_msg.pose.pose.orientation.z = heading_quat[2]
    gnss_odom_msg.pose.pose.orientation.w = heading_quat[3]

    # reset_pub.publish(gnss_odom_msg)

    # Publish TF transform from `odom` to `base_link`
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_link"
    transform.transform.translation.x = coords[0].item()
    transform.transform.translation.y = coords[1].item()
    transform.transform.translation.z = 0
    transform.transform.rotation.x = imu_quat[0]
    transform.transform.rotation.y = imu_quat[1]
    transform.transform.rotation.z = imu_quat[2]
    transform.transform.rotation.w = imu_quat[3]

    tf_broadcaster.sendTransform(transform)
    gnss_pub.publish(gnss_odom_msg)


def listener():
    global reset_pub, tf_broadcaster, gnss_pub
    rospy.init_node('lla_to_odom', anonymous=True)

    # Subscribers
    rospy.Subscriber('/reach/fix', NavSatFix, gnss_callback)
    # rospy.Subscriber('/filter/quaternion', Imu, imu_callback)
    rospy.Subscriber('/heading/filtered', Odometry, localization_callback)

    # Publishers
    gnss_pub = rospy.Publisher('/odometry/lla2odom', Odometry, queue_size=10)
    fix_status_pub = rospy.Publisher('/fix_status', Int32, queue_size=10)
    reset_pub = rospy.Publisher('set_pose', PoseWithCovarianceStamped, queue_size=10)

    # Initialize TF Broadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        
        fix_status_msg = Int32()
        fix_status_msg.data = fix_status
        fix_status_pub.publish(fix_status_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Node Interrupted. Exiting.")
