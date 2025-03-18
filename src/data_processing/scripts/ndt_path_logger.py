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

file_path = f"/home/catkin_ws/src/data_processing/data/online_plotting/run_no{run_no}/ndt_odom.csv"
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

    msg.header.frame_id = "map"
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
    # reset_msg = PoseWithCovarianceStamped()
    # # reset_msg.pose.pose = msg.pose
    # reset_msg.header.stamp = msg.header.stamp

    # reset_msg.header.frame_id = "odom"  # Replace with your localization frame, e.g., "odom" or "base_link"

    # # Set the desired pose (x, y, z, orientation as quaternion)
    # reset_msg.pose.pose.position.x = msg.pose.position.x
    # reset_msg.pose.pose.position.y = msg.pose.position.y
    # reset_msg.pose.pose.position.z = 0.0

    # r, p, curr_yaw  = tf.euler_from_quaternion(quaternion)
    # relative_yaw = curr_yaw - yaw
    # qx, qy, qz, qw = tf.quaternion_from_euler(0, 0, relative_yaw)

    # reset_msg.pose.pose.orientation.x = qx
    # reset_msg.pose.pose.orientation.y = qy
    # reset_msg.pose.pose.orientation.z = qz
    # reset_msg.pose.pose.orientation.w = qw
    # reset_msg.pose.covariance = [
    #     0.00001, 0, 0, 0, 0, 0,
    #     0, 0.00001, 0, 0, 0, 0,
    #     0, 0, 0.00001, 0, 0, 0,
    #     0, 0, 0, 0.00001, 0, 0,
    #     0, 0, 0, 0, 0.00001, 0,
    #     0, 0, 0, 0, 0, 0.00001   
    # ]

        

    # Log the data to the CSV file
    with open(file_path, mode='a') as f:
        writer = csv.writer(f)
        writer.writerow([timestamp, x, y])
    f.close()


def listener():
    global poses, last_reset_time, reset_msg
    rospy.init_node('path_listener', anonymous=True)
    rospy.Subscriber('/ndt_pose', PoseStamped, ndt_callback)
    pub = rospy.Publisher('/path', Path)
    reset_pub = rospy.Publisher('set_pose', PoseWithCovarianceStamped)
    rate = rospy.Rate(10)
    last_reset_time = rospy.get_time()
    while not rospy.is_shutdown():
        curr_time = rospy.get_time()
        dur = (curr_time - last_reset_time)
        # if dur > 10 and reset_msg is not None:
        #     print("resetting!!!")
        #     print(reset_msg)
        #     reset_pub.publish(reset_msg)
        #     reset_msg = None
        #     last_reset_time = curr_time
        pub_msg = Path()
        pub_msg.header.frame_id = "map"
        pub_msg.poses = poses
        pub.publish(pub_msg)
        rate.sleep()

def shutdown():
    global run_no
    with open("/home/catkin_ws/src/data_processing/data/online_plotting/run_no.txt", mode = 'w') as f:
        f.write(str(run_no + 1))
    f.close()
    directory = f"/home/catkin_ws/src/data_processing/data/online_plotting/run_no{run_no+1}"
    if not os.path.exists(directory):
        os.makedirs(directory)
    
    

if __name__ == '__main__':
    try:
        listener()
        rospy.on_shutdown(shutdown)
    except rospy.ROSInterruptException:
        print("exiting")
