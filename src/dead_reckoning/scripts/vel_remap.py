#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64 
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

def vector3Callback(msg):
    global re_pub

    repub_msg = TwistStamped()
    repub_msg.twist.linear.x = msg.twist.twist.linear.x * 0.22
    repub_msg.twist.angular.z = msg.twist.twist.angular.z
    
    re_pub.publish(repub_msg)

def main():
    global re_pub
    print("init")

    rospy.init_node('vel_convert')
    rospy.Subscriber('/bicycle/odom', Odometry, vector3Callback)
    re_pub = rospy.Publisher('/amr50/velocity', TwistStamped, queue_size=10)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass