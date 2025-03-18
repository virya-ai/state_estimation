#!/usr/bin/env python3
import rospy
import csv
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
import os 
import numpy as np 
import tf.transformations as tf
import copy

transformation_matrix = np.zeros(shape=[3,3])
poses = []
iter = 0
run_no = 0
with open("/home/catkin_ws/src/data_processing/data/online_plotting/run_no.txt", mode = 'r') as f:
    run_no = int(f.read())
f.close()

file_path = f"/home/catkin_ws/src/data_processing/data/online_plotting/run_no{run_no}/ndt_odom_temp.csv"
with open(file_path, mode='w') as f:
    writer = csv.writer(f)
    writer.writerow(["timestamp", "x", "y"])
f.close()

last_reset_time = 0
reset_msg = None

yaw = 0

def ndt_callback(msg):
    global reset_msg 
    global writer
    global file_path
    global poses
    global yaw
    global transformation_matrix
    """
    Callback function for the Odometry topic. Logs x and y coordinates to CSV.
    """
    # Extract x, y, and timestamp from the Odometry message
    x = msg.pose.position.x 
    y = msg.pose.position.y 
    timestamp = rospy.get_time()

    msg.header.frame_id = "camera_init"
    # quaternion = (
    #         msg.pose.orientation.x,
    #         msg.pose.orientation.y,
    #         msg.pose.orientation.z,
    #         msg.pose.orientation.w,
    #     )
    # if (len(poses) == 0):
        
        
    #     _, _, yaw = tf.euler_from_quaternion(quaternion)
    #     transformation_matrix = np.linalg.inv( np.array([[np.cos(yaw), -np.sin(yaw), x], [np.sin(yaw), np.cos(yaw), y], [0, 0, 1]]))
        
    # state = np.array([x, y, 1]).reshape([3, 1])
    # transformed =  np.matmul(transformation_matrix, state)  
    # x = msg.pose.position.x = transformed[0].item()
    # y = msg.pose.position.y = transformed[1].item()

    poses.append(msg)

    # Log the data to the CSV file
    with open(file_path, mode='a') as f:
        writer = csv.writer(f)
        writer.writerow([timestamp, x, y])
    f.close()


def listener():
    global poses, last_reset_time, reset_msg
    rospy.init_node('path_listener', anonymous=True)
    rospy.Subscriber('/ndt_pose_temp', PoseStamped, ndt_callback)
    pub = rospy.Publisher('temp/path', Path)
    rate = rospy.Rate(10)
    last_reset_time = rospy.get_time()
    while not rospy.is_shutdown():
        pub_msg = Path()
        pub_msg.header.frame_id = "camera_init"
        pub_msg.poses = poses
        pub.publish(pub_msg)
        rate.sleep()



if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        print("exiting")
