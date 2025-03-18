#!/usr/bin/env python2
import rospy
import math
import numpy as np
from collections import deque
import tf.transformations as tf
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from utils import GPS_utils

# Global Variables
gps_converter = GPS_utils()
gnss_odom_msg = Odometry()
fix_status = 0
prev_quat = [0, 0, 0, 1]

# Set the ENU origin
lat = 12.818808815442026
long = 77.69415522995406
alt = 815.1511848832016
gps_converter.setENUorigin(lat, long, alt)
prev_x = None
prev_y = None

# Store past n=10 coordinates
x_coords = deque(maxlen=5)
y_coords = deque(maxlen=5)

# ROS Publishers
reset_pub = None  # Will be initialized in listener()


def fit_line_r2(x, y):
    """
    Fit a line to x, y points and return R (coefficient of determination).
    If R is close to 1, points are in a straight line.
    """
    if len(x) < 2:
        return 0  # Not enough points

    x = np.array(x, dtype=np.float64)
    y = np.array(y, dtype=np.float64)
    # Fit line using numpy polyfit (degree 1 for straight line)
    # print "y" ,e y
    
    A = np.vstack([x, np.ones(len(x))]).T
    # print "A", A
    m, c = np.linalg.lstsq(A, y, rcond=-1)[0]  # y = mx + c

    # Compute R (coefficient of determination)
    y_pred = m * np.array(x) + c
    ss_tot = np.sum((y - np.mean(y)) ** 2)
    ss_res = np.sum((y - y_pred) ** 2)
    r2 = 1 - (ss_res / ss_tot if ss_tot != 0 else 1)

    return r2


def compute_orientation(x, y):
    """
    Computes the heading from a list of x, y coordinates.
    Uses average heading between consecutive points.
    """
    global prev_quat
    if len(x) < 2:
        return prev_quat  # Not enough data

    headings = []
    for i in range(len(x) - 1):
        dx = x[i + 1] - x[i]
        dy = y[i + 1] - y[i]
        delta_pose = np.sqrt(dx ** 2 + dy ** 2)
        if delta_pose > 0.05:
            headings.append(math.atan2(dy, dx))

    if len(headings) < 5:
        return prev_quat

    avg_heading = math.atan2(np.mean(np.sin(headings)), np.mean(np.cos(headings)))
    quaternion = tf.quaternion_from_euler(0, 0, avg_heading)

    return quaternion


def gnss_callback(data):
    global gnss_odom_msg, fix_status, x_coords, y_coords, prev_quat, reset_pub

    # Convert GPS to ENU
    lat, lon, alt = data.latitude, data.longitude, data.altitude
    fix_status = data.status.status
    coords = gps_converter.geo2enu(lat, lon, alt)

    # Store coordinates
    x_coords.append(coords[0].item())
    y_coords.append(coords[1].item())

    # Compute orientation
    quat = compute_orientation(x_coords, y_coords)
    prev_quat = quat

    # Update odometry message
    gnss_odom_msg = Odometry()
    gnss_odom_msg.header.stamp = rospy.Time.now()
    gnss_odom_msg.header.frame_id = "odom"
    gnss_odom_msg.pose.pose.position.x = coords[0].item()
    gnss_odom_msg.pose.pose.position.y = coords[1].item()
    gnss_odom_msg.pose.pose.position.z = 0
    gnss_odom_msg.pose.pose.orientation.x = quat[0]
    gnss_odom_msg.pose.pose.orientation.y = quat[1]
    gnss_odom_msg.pose.pose.orientation.z = quat[2]
    gnss_odom_msg.pose.pose.orientation.w = quat[3]

    if prev_x == None:
        prev_x = coords[0].item()
        prev_y = coords[1].item()

    dx = coords[0].item() - prev_x 
    dy = coords[1].item() - prev_y
    delta = np.sqrt(dx **2 + dy ** 2 )
    if delta > 1:  # Only check when we have 10 points
        reset_msg = PoseWithCovarianceStamped()
        reset_msg.header.stamp = rospy.Time.now()
        reset_msg.header.frame_id = "odom"
        reset_msg.pose.pose.position.x = coords[0].item()
        reset_msg.pose.pose.position.y = coords[1].item()
        reset_msg.pose.pose.position.z = 0
        reset_msg.pose.pose.orientation.x = quat[0]
        reset_msg.pose.pose.orientation.y = quat[1]
        reset_msg.pose.pose.orientation.z = quat[2]
        reset_msg.pose.pose.orientation.w = quat[3]

        reset_pub.publish(reset_msg)



def listener():
    global reset_pub
    rospy.init_node('lla_to_odom', anonymous=True)
    rospy.Subscriber('/reach/fix', NavSatFix, gnss_callback)

    gnss_pub = rospy.Publisher('/odometry/gps', Odometry, queue_size=10)
    fix_status_pub = rospy.Publisher('/fix_status', Int32, queue_size=10)
    reset_pub = rospy.Publisher('set_pose', PoseWithCovarianceStamped, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        gnss_pub.publish(gnss_odom_msg)
        
        fix_status_msg = Int32()
        fix_status_msg.data = fix_status
        fix_status_pub.publish(fix_status_msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Node Interrupted. Exiting.")
