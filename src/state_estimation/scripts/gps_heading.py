#!/usr/bin/env python2
 
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import tf.transformations as tf
import math
import numpy as np 
import time
from collections import deque

class HeadingEstimator:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('heading_estimator', anonymous=True)
 
        # Subscriber to the /gps/filtered topic
        self.gps_sub = rospy.Subscriber('/odometry/filtered', Odometry, self.gps_callback)
 
        # Publisher for the estimated heading
        self.heading_pub = rospy.Publisher('/heading/gps', Odometry, queue_size=10)
 
        # Variables to store previous GPS data
        self.prev_x = 0
        self.prev_y = 0
        self.prev_time = rospy.Time.now().to_sec()
        self.heading = 0

        self.vx = 0 
        self.vy = 0

        self.x_coords = deque(maxlen=4)
        self.y_coords = deque(maxlen=4)
        self.times = deque(maxlen=10)
        self.headings = deque(maxlen=10)
        self.headings.append(0)

    def get_heading(self):
        
        if len(self.x_coords) > 1 and len(self.y_coords) > 1 and len(self.times) > 1:
            delta_x = self.x_coords[-1] - self.x_coords[-2]
            delta_y = self.y_coords[-1] - self.y_coords[-2]
            delta_t = self.times[-1] - self.times[-2]

            if delta_t > 0:
                self.vx = delta_x / delta_t
                self.vy = delta_y / delta_t

            heading = np.arctan2(self.vy, self.vx)

            self.headings.append(heading)
            if len(self.headings) == 10 and len(self.times) == 10:
                times = np.array(self.times)
                headings = np.array(self.headings)
                coeffs = np.polyfit(times, headings, 3)
                poly = np.poly1d(coeffs)
                future_time = self.times[-1] + 0.25
                future_heading = poly(future_time)
                self.heading = future_heading


    def gps_callback(self, msg):
        # print("gps callback")
        # Extract the x and y coordinates from the message
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        # Check if we have previous data to compute heading
        delta_x = current_x - self.prev_x
        delta_y = current_y - self.prev_y
        if np.sqrt(delta_x**2 + delta_y**2) > 0.2:
            # Calculate the difference in latitude and longitude   
            # print("delta_x: ", delta_x)
            # print("delta_y: ", delta_y)
            self.x_coords.append(current_x)
            self.y_coords.append(current_y)
            self.times.append(rospy.Time.now().to_sec())

            self.get_heading()
            
            # Update previous data with current data
            self.prev_x = current_x
            self.prev_y = current_y
            

    def publish_heading(self):
        # Publish the heading to the /gps/heading topic
        heading_msg = Odometry()
        heading_msg.header.stamp = rospy.Time.now()
        heading_msg.header.frame_id = "odom"
        heading_msg.child_frame_id = "imu_link"
        heading_quat = tf.quaternion_from_euler(0, 0,self.heading)
        heading_msg.pose.pose.orientation.x = heading_quat[0]
        heading_msg.pose.pose.orientation.y = heading_quat[1]
        heading_msg.pose.pose.orientation.z = heading_quat[2]
        heading_msg.pose.pose.orientation.w = heading_quat[3]
        
        self.heading_pub.publish(heading_msg)

 
if __name__ == '__main__':
    try:
        # Create an instance of the HeadingEstimator class
        estimator = HeadingEstimator()
        rate = rospy.Rate(20)  # 10 Hz
        while not rospy.is_shutdown():
            # Spin to keep the script running and processing callbacks
            estimator.publish_heading()
            rate.sleep()
 
        # Spin to keep the script running and processing callbacks
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
 