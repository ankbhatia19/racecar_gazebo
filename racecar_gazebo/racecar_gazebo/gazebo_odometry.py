#!/usr/bin/env python3

'''
This script makes Gazebo less fail by translating gazebo status messages to odometry data.
Since Gazebo also publishes data faster than normal odom data, this script caps the update to 20hz.
Winter Guerra
'''

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import math
import tf2_ros

class OdometryNode(Node):

    def __init__(self):
        super().__init__('gazebo_odometry_node')
        # init internals
        self.last_received_pose = Pose()
        self.last_received_twist = Twist()
        self.last_recieved_stamp = None

        # Set the update rate
        self.tmr = self.create_timer(.05, self.timer_callback) # 20hz

        self.tf_pub = tf2_ros.TransformBroadcaster(self)

        # Set publishers
        self.pub_odom = self.create_publisher(Odometry,'/vesc/odom', 1)

        # Set subscribers
        self.sub = self.create_subscription(LinkStates,'/gazebo/link_states', self.sub_robot_pose_update, 10)

    def sub_robot_pose_update(self, msg):
        # Find the index of the racecar
        try:
            arrayIndex = msg.name.index('racecar::base_link')
        except ValueError as e:
            # Wait for Gazebo to startup
            pass
        else:
            # Extract our current position information
            self.last_received_pose = msg.pose[arrayIndex]
            self.last_received_twist = msg.twist[arrayIndex]
        self.last_recieved_stamp = rclpy.Time.now()

    def timer_callback(self):
        if self.last_recieved_stamp is None:
            return

        cmd = Odometry()
        cmd.header.stamp = self.last_recieved_stamp
        cmd.header.frame_id = 'odom'
        cmd.child_frame_id = 'base_link'
        cmd.pose.pose = self.last_received_pose
        cmd.twist.twist = self.last_received_twist
        self.pub_odom.publish(cmd)

        tf = TransformStamped(
            header=Header(
                frame_id=cmd.header.frame_id,
                stamp=cmd.header.stamp
            ),
            child_frame_id=cmd.child_frame_id,
            transform=Transform(
                translation=cmd.pose.pose.position,
                rotation=cmd.pose.pose.orientation
            )
        )
        self.tf_pub.sendTransform(tf)

# Start the node
def main(args=None):
    rclpy.init(args=args)

    node = OdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()