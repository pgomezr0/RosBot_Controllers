
'''
Created 29 Nov 2021
@author: pgomezr0
'''

# INTELLIGENT SYSTEMS AND ROBOTICS ASSIGNMENT 1
# PID CONTROLLER
# Author: Paola Gomez Reyna

import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

i_error = []
prev_error = 0.0


class MinimalSubsPublisher(Node):
    def __init__(self):
        super().__init__('PID')
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, qos_profile_sensor_data)
        time.sleep(10)
        # Initializes list to insert sensor values
        self.laser_ranges = [0.0 for i in range(1440)]
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd = Twist()
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def PID(self, k_p, k_i, k_d):

        # Right Edge Following
        global i_error
        global prev_error

        distances = self.laser_ranges[1079:1440]  # Right edge distance to wall

        if len(distances) != 0:
            minimum_distance = min(distances)
        else:
            minimum_distance = 0.0

        # Deal with infinities
        if minimum_distance > 1.0:
            minimum_distance = 1.0

        # Error
        desired_distance = 0.5
        error = desired_distance - minimum_distance

        # I Error
        if len(i_error) > 9:  # Last 10 errors
            i_error.pop(0)
        i_error.append(error)
        i_error_sum = sum(i_error)

        # D Error
        d_error = error - prev_error

        # Prev Error
        prev_error = error

        turning_output = (error*k_p) + (i_error_sum*k_i) + (d_error*k_d)

        self.get_logger().info('error: %f, i_error: %f, d_error: %f,' %
                               (error, i_error_sum, d_error))

        self.cmd.linear.x = 0.2
        self.cmd.angular.z = turning_output

    def timer_callback(self):
        self.PID(1.0, 0.1, 5)
        self.publisher_.publish(self.cmd)
        self.get_logger().info('Speed: %f, Steering: %.5f' %
                               (self.cmd.linear.x, self.cmd.angular.z))
        self.get_logger().info('*************************')

    def laser_callback(self, msg):
        self.laser_ranges = msg.ranges


def main(args=None):

    rclpy.init(args=args)
    subs_publisher = MinimalSubsPublisher()
    rclpy.spin(subs_publisher)

    # Destroy Node
    subs_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
