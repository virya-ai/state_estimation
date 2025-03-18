#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PoseToPathConverter:
    def __init__(self):
        # Initialize the node
        rospy.init_node('pose_to_path_converter', anonymous=True)

        # Publisher for the path
        self.path_pub = rospy.Publisher('/path', Path, queue_size=1)

        # Subscriber to the pose topic
        self.pose_sub = rospy.Subscriber('/ndt_pose1', PoseStamped, self.pose_callback)

        # Initialize the path message
        self.path = Path()
        self.path.header.frame_id = "map"  # Ensure a fixed frame ID

        # Limit the number of stored poses
        self.max_path_size = 500

    def pose_callback(self, pose_msg):
        # Ensure the frame_id is consistent
        pose_msg.header.frame_id = self.path.header.frame_id

        # Append the pose to the path
        self.path.poses.append(pose_msg)

        # Limit the size of the stored pathi
        if len(self.path.poses) > self.max_path_size:
            self.path.poses.pop(0)

        # Update the header timestamp
        self.path.header.stamp = rospy.Time.now()

        # Publish the path
        self.path_pub.publish(self.path)

if __name__ == '__main__':
    try:
        converter = PoseToPathConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
