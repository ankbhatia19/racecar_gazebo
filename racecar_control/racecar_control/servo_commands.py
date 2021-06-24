#!/usr/bin/env python3
import rclpy
import sys
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from ackermann_msgs.msg import AckermannDriveStamped

flag_move = 0

rclpy.init(args=sys.argv)
node = rclpy.create_node('servo_commands')
node.get_logger().info('Created node')

pub_vel_left_rear_wheel = node.create_publisher(Float64MultiArray,'/left_rear_wheel_velocity_controller/commands',5)
pub_vel_right_rear_wheel = node.create_publisher(Float64MultiArray,'/right_rear_wheel_velocity_controller/commands',5)
pub_vel_left_front_wheel = node.create_publisher(Float64MultiArray,'/left_front_wheel_velocity_controller/commands',5)
pub_vel_right_front_wheel = node.create_publisher(Float64MultiArray,'/right_front_wheel_velocity_controller/commands',5)

# TODO: This still is not connected to the controller
pub_pos_left_steering_hinge = node.create_publisher(Float64MultiArray,'/left_steering_hinge_position_controller/commands',5)
pub_pos_right_steering_hinge = node.create_publisher(Float64MultiArray,'/right_steering_hinge_position_controller/commands',5)


def set_throttle_steer(data):

    global flag_move
    throttle = data.drive.speed/0.1
    steer = data.drive.steering_angle

    pub_vel_left_rear_wheel.publish(Float64MultiArray(data=[throttle]))
    pub_vel_right_rear_wheel.publish(Float64MultiArray(data=[throttle]))
    pub_vel_left_front_wheel.publish(Float64MultiArray(data=[throttle]))
    pub_vel_right_front_wheel.publish(Float64MultiArray(data=[throttle]))
    
    pub_pos_left_steering_hinge.publish(Float64MultiArray(data=[steer]))
    pub_pos_right_steering_hinge.publish(Float64MultiArray(data=[steer]))

def servo_commands(args=sys.argv):

    node.create_subscription(AckermannDriveStamped,"/racecar/ackermann_cmd_mux/output", set_throttle_steer,qos_profile=qos_profile_sensor_data)

    # spin() simply keeps python from exiting until this node is stopped
    rclpy.spin(node)

if __name__ == '__main__':
    try:
        servo_commands()
    except:
        pass
