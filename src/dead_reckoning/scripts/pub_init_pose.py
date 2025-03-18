#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import tf.transformations as tf


init_msg = PoseStamped
init_flag = False
init_pose_msg = PoseWithCovarianceStamped()

def init_pose_cb(msg):
    global init_msg, init_flag
    if not init_flag:
        init_msg = msg
        init_flag = True
        init_pose_msg.pose.pose = msg.pose
        init_pose_msg.header.frame_id = "map"
        


def main(): 
    rospy.init_node('pub_initial_pose')
    init_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    init_pose_sub = rospy.Subscriber('/ndt_pose', PoseStamped, init_pose_cb)
    rate = rospy.Rate(3)
    
    while not rospy.is_shutdown():
        if init_flag == True:
            init_pose_pub.publish(init_pose_msg)
            print("publishing init pose!!")
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass