#!/usr/bin/env python2
import rospy
import csv
import os
import numpy as np
import tf.transformations as tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

# Global variables
poses = []
yaw = 0
last_reset_time = 0
reset_msg = None
pub = None

# Read run number from file
run_no = 0
run_no_file = "/home/catkin_ws/src/data_processing/data/online_plotting/run_no.txt"
if os.path.exists(run_no_file):
    with open(run_no_file, 'r') as f:
        run_no = int(f.read().strip())

# Ensure directory exists
csv_dir = "/home/catkin_ws/src/data_processing/data/online_plotting/run_no{}".format(run_no)
if not os.path.exists(csv_dir):
    os.makedirs(csv_dir)

# CSV file setup
csv_file = os.path.join(csv_dir, "ndt_odom.csv")
with open(csv_file, 'w') as f:
    writer = csv.writer(f)
    writer.writerow(["timestamp", "x", "y"])

def ndt_callback(msg):
    """Callback function for the Odometry topic. Logs x and y coordinates to CSV."""
    global pub

    # Republish as Odometry
    repub_msg = Odometry()
    repub_msg.header = msg.header
    repub_msg.pose.pose = msg.pose

    # Publish to /odom topic
    pub.publish(repub_msg)

    # Extract position and timestamp
    x, y = msg.pose.position.x, msg.pose.position.y
    timestamp = msg.header.stamp.to_sec()

    # Log data to CSV
    with open(csv_file, 'a') as f:
        writer = csv.writer(f)
        writer.writerow([timestamp, x, y])


def listener():
    """Initializes the ROS node and subscribes to topics."""
    global pub

    rospy.init_node('path_listener', anonymous=True)

    # Subscribers
    rospy.Subscriber('/ndt_pose', PoseStamped, ndt_callback, queue_size=10)

    # Publisher with explicit queue size (needed in Melodic)
    pub = rospy.Publisher('/odom', Odometry, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        rospy.loginfo("Path listener node exiting.")
