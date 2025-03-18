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
configure_flag = False
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
down_sample_rate = 10
counter = 0

yaw = 0

last_x = None
last_y = None 
net_distance = 0
reset_pub = None
def ndt_callback(msg):
    global configure_flag
    global reset_msg 
    global writer
    global file_path
    global poses
    global yaw
    global transformation_matrix
    global counter
    global down_sample_rate
    global last_x
    global last_y
    global net_distance
    global reset_pub
    """
    Callback function for the Odometry topic. Logs x and y coordinates to CSV.
    """
    # Extract x, y, and timestamp from the Odometry message
    x = msg.pose.position.x 
    y = msg.pose.position.y 
    msg.pose.position.z = 0
    if last_x == None and last_y == None:
        last_x = x
        last_y = y

    timestamp = rospy.get_time()

    msg.header.frame_id = "map"

    if (counter % down_sample_rate == 0):
        last_x = x
        last_y = y
        poses.append(copy.deepcopy(msg))
        poses.append(msg)
        # if len(poses) > 100:
        #     while len(poses) > 100:
        #         poses.pop(0)
        with open(file_path, mode='a') as f:
            writer = csv.writer(f)
            writer.writerow([timestamp, x, y])
        f.close()
    counter += 1




def listener():
    global poses, last_reset_time, reset_msg, reset_pub
    rospy.init_node('path_listener', anonymous=True)
    rospy.Subscriber('/ndt_pose', PoseStamped, ndt_callback)
    pub = rospy.Publisher('/path', Path)
    reset_pub = rospy.Publisher('set_pose', PoseWithCovarianceStamped)
    rate = rospy.Rate(10)
    last_reset_time = rospy.get_time()
    while not rospy.is_shutdown():
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
