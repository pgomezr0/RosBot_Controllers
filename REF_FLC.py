
'''
Created 3 Dec 2021
@author: pgomezr0
'''

# INTELLIGENT SYSTEMS AND ROBOTICS ASSIGNMENT 1
# RIGHT EDGE FOLLOWING FUZZY LOGIC CONTROLLER
# Author: Paola Gomez Reyna

import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from collections import defaultdict


class MinimalSubsPublisher(Node):
    def __init__(self):
        super().__init__('REF_FLC')
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, qos_profile_sensor_data)
        time.sleep(10)
        # Initializes list to insert sensor values
        self.laser_ranges = [0.0 for i in range(1440)]
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd = Twist()
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def fuzzy_logic_right_edge(self):

        # Nearest to collision from the right
        RB_distance = min(self.laser_ranges[859:1100])  # Right Back
        RF_distance = min(self.laser_ranges[1100:1341])  # Right Front

        print(RB_distance, RF_distance)

        # Deal with infinite values
        if RB_distance > 1.0:
            RB_distance = 1.0
        if RF_distance > 1.0:
            RF_distance = 1.0

        # Membership Values

        # Right Back
        near_dist_RB = [0.0, 0.0, 0.25, 5]
        self.near_RB = self.get_membership_value(RB_distance, near_dist_RB)
        med_dist_RB = [0.25, 0.5, 0.5, 0.75]
        self.med_RB = self.get_membership_value(RB_distance, med_dist_RB)
        far_dist_RB = [0.5, 0.75, 0.75, 1.0]
        self.far_RB = self.get_membership_value(RB_distance, far_dist_RB)

        # Right Front
        near_dist_RF = [0.0, 0.0, 0.25, 5]
        self.near_RF = self.get_membership_value(RF_distance, near_dist_RF)
        med_dist_RF = [0.25, 0.5, 0.5, 0.75]
        self.med_RF = self.get_membership_value(RF_distance, med_dist_RF)
        far_dist_RF = [0.5, 0.75, 0.75, 1.0]
        self.far_RF = self.get_membership_value(RF_distance, far_dist_RF)

        # Output Velocity
        slow_vel = [0.04, 0.08, 0.12, 0.16]
        # In case of future tests with different iputs of velocity
        self.slow = self.calculate_centroid(slow_vel)
        # self.slow = 0.1

        med_vel = [0.08, 0.16, 0.24, 0.3]
        self.med = self.calculate_centroid(med_vel)
        # self.med = 0.195

        fast_vel = [0.12, 0.24, 0.36, 0.48]
        self.fast = self.calculate_centroid(fast_vel)
        # self.fast = 0.3

        # Output Turning (Steering)
        # right_turning = [-0.4, -0.6, -0.8, -1.0]
        # self.right = self.calculate_centroid(right_turning)
        self.right = -0.5

        self.no_turn = 0

        # left_turning = [0.4, 0.6, 0.8, 1.0]
        # self.left = self.calculate_centroid(left_turning)
        self.left = 0.5

        rules_dict = self.set_rules()

        LMS_num = []
        LMS_denom = []
        RMS_num = []
        RMS_denom = []

        # For each rule will set membership values
        for rule in rules_dict.values():
            value_1 = rule[0][0]
            value_2 = rule[0][1]
            value_3 = rule[0][2]  # Turning COM
            value_4 = rule[0][3]  # Speed COM

            firing_strength = min(value_1, value_2)

            if firing_strength > 0:
                LMS = value_3
                RMS = value_4

                # Defuzzing / Calculate speed and steering outputs
                LMS_num.append(firing_strength*LMS)
                RMS_num.append(firing_strength*RMS)
                LMS_denom.append(firing_strength)
                RMS_denom.append(firing_strength)

                rule_fired = str(list(rules_dict.keys())[
                                 list(rules_dict.values()).index(rule)])
                self.get_logger().info('Rule fired: ' + str(rule_fired))
            else:
                pass  # Rule not applied

        # Motor Turning and speed
        LMS = sum(LMS_num) / sum(LMS_denom)
        RMS = sum(RMS_num) / sum(RMS_denom)

        self.cmd.angular.z = LMS  # Steering
        self.cmd.linear.x = RMS  # Speeds

    def get_membership_value(self, real_distance, points_list):
        # Rising shape
        if real_distance >= points_list[0] and real_distance < points_list[1]:
            return (real_distance - points_list[0]) / (points_list[1] - points_list[0])

        # Plain shape
        elif real_distance >= points_list[1] and real_distance <= points_list[2]:
            return 1.0

        # Falling shape
        elif real_distance > points_list[2] and real_distance < points_list[3]:
            return (points_list[3] - real_distance) / (points_list[3] - points_list[2])

        return 0.0

    def calculate_centroid(self, points):
        return sum(points) / len(points)

    def set_rules(self):
        rules = defaultdict(list)

        #                    [ RF ]      [ RB ]     [ Turning ] [ Speed ]
        rules[0].append([self.near_RF, self.near_RB, self.left, self.slow])
        rules[1].append([self.near_RF, self.med_RB, self.left, self.slow])
        rules[2].append([self.near_RF, self.far_RB, self.left, self.slow])

        rules[3].append([self.med_RF, self.near_RB, self.right, self.slow])
        rules[4].append([self.med_RF, self.med_RB, self.no_turn, self.med])
        rules[5].append([self.med_RF, self.far_RB, self.left, self.med])

        rules[6].append([self.far_RF, self.near_RB, self.right, self.slow])
        rules[7].append([self.far_RF, self.med_RB, self.right, self.med])
        rules[8].append([self.far_RF, self.far_RB, self.right, self.fast])

        return rules

    def timer_callback(self):
        self.fuzzy_logic_right_edge()
        self.publisher_.publish(self.cmd)
        self.get_logger().info('Speed: %.5f, Steering: %.5f' %
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
