#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import MagneticField

def vector3Callback(msg):
    global mag_pub

    mag_msg = MagneticField()
    mag_msg.header = msg.header
    mag_msg.magnetic_field = msg.vector
    
    mag_pub.publish(mag_msg)

def main():
    global mag_pub

    rospy.init_node('mag_convert')
    rospy.Subscriber('/imu/mag_raw', Vector3Stamped, vector3Callback)
    mag_pub = rospy.Publisher('/imu/mag', MagneticField, queue_size=10)
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass