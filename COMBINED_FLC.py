'''
Created 10 Dec 2021
@author: pgomezr0
'''

# INTELLIGENT SYSTEMS AND ROBOTICS ASSIGNMENT 1
# COMBINED FUZZY LOGIC CONTROLLER
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
        super().__init__('COMBINED_FLC')
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, qos_profile_sensor_data)
        time.sleep(10)
        # Initializes list to insert sensor values
        self.laser_ranges = [0.0 for i in range(1440)]
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd = Twist()
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def higher_fuzzification(self):

        # Nearest to collision from front
        RF_distance = min(self.laser_ranges[1200:1380])  # Right Front
        F_distance = min(min(self.laser_ranges[1380:1440]), min(
            self.laser_ranges[0:60]))  # Front
        LF_distance = min(self.laser_ranges[60:240])  # Left Front
        minimum_distance = min(RF_distance, F_distance, LF_distance)

        # Deal with infinite values
        if RF_distance > 1.2:
            RF_distance = 1.2
        if F_distance > 1.2:
            F_distance = 1.2
        if LF_distance > 1.2:
            LF_distance = 1.2

        # Boundaries for Obstacle Avoidance and Right Edge Following
        limit_REF = 0.6
        limit_OA = 0.3

        if (minimum_distance > limit_OA) and (minimum_distance < limit_REF):  # Between limits

            member_values = [0 for i in range(2)]

            # Obtain manually membership values and create shape

            member_values[0] = (minimum_distance - limit_OA) / \
                (limit_REF - limit_OA)
            member_values[1] = (limit_REF / minimum_distance) / \
                (limit_REF - limit_OA)

            values_OA = list(self.obstacle_avoidance())
            values_REF = list(self.right_edge_following())

            for i in range(len(values_OA)):
                values_OA[i] = values_OA[i] * member_values[0]
                values_REF[i] = values_REF[i] * member_values[1]

            # Motor Turning and speed
            denom = member_values[0] + member_values[1]
            LMS_h = (values_OA[0] + values_REF[0]) / denom  # Steering
            RMS_h = (values_OA[1] + values_REF[1]) / denom  # Speed

            self.cmd.angular.z = LMS_h  # Steering
            self.cmd.linear.x = RMS_h  # Speed

        elif minimum_distance <= limit_OA:
            LMS_OA, RMS_OA = self.obstacle_avoidance()
            self.cmd.angular.z = LMS_OA  # Steering
            self.cmd.linear.x = RMS_OA  # Speed

        else:
            LMS_REF, RMS_REF = self.right_edge_following()
            self.cmd.angular.z = LMS_REF  # Steering
            self.cmd.linear.x = RMS_REF  # Speed

        return

    def right_edge_following(self):
        # Nearest to collision from the right
        # Right Back [1100:1341]
        RB_distance = min(self.laser_ranges[859:1100])
        # Right Front [859:1100]
        RF_distance = min(self.laser_ranges[1100:1341])

        print(RB_distance, RF_distance)

        # Deal with infinite values
        if RB_distance > 1.0:
            RB_distance = 1.0
        if RF_distance > 1.0:
            RF_distance = 1.0

        # Membership Values

        # Right Back
        near_dist_RB = [0.0, 0.0, 0.25, 0.5]
        self.near_RB = self.get_membership_value(RB_distance, near_dist_RB)
        med_dist_RB = [0.25, 0.5, 0.5, 0.75]
        self.med_RB = self.get_membership_value(RB_distance, med_dist_RB)
        far_dist_RB = [0.5, 0.75, 0.75, 1.0]
        self.far_RB = self.get_membership_value(RB_distance, far_dist_RB)

        # Right Front
        near_dist_RF = [0.0, 0.0, 0.25, 0.5]
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
        # right_turning = [-0.3, -0.5, -0.8, -1.0]
        # self.right = self.calculate_centroid(right_turning)
        self.right = -0.5

        self.no_turn = 0

        # left_turning = [0.4, 0.6, 0.8, 1.0]
        # self.left = self.calculate_centroid(left_turning)
        self.left = 0.5

        rules_dict_REF = self.set_rules_REF()

        LMS_num = []
        LMS_denom = []
        RMS_num = []
        RMS_denom = []

        # For each rule will set membership values
        for rule in rules_dict_REF.values():
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

                rule_fired = str(list(rules_dict_REF.keys())[
                                 list(rules_dict_REF.values()).index(rule)])
                self.get_logger().info('Rule fired: ' + str(rule_fired))
            else:
                pass  # Rule not applied

        # Motor Turning and speed
        LMS = sum(LMS_num) / sum(LMS_denom)
        RMS = sum(RMS_num) / sum(RMS_denom)

        return(LMS, RMS)

    def obstacle_avoidance(self):

        # Distances nearest to collision
        RF_distance = min(self.laser_ranges[1200:1380])  # Right Front
        F_distance = min(min(self.laser_ranges[1380:1440]), min(
            self.laser_ranges[0:60]))  # Front
        LF_distance = min(self.laser_ranges[60:240])  # Left Front

        # Deal with infinite values
        if RF_distance > 1.2:
            RF_distance = 1.2
        if F_distance > 1.2:
            F_distance = 1.2
        if LF_distance > 1.2:
            LF_distance = 1.2

        print(RF_distance, F_distance, LF_distance)

        # Membership Values
        # Right Front
        near_dist_RF = [0.0, 0.0, 0.3, 0.6]
        self.near_RF = self.get_membership_value(RF_distance, near_dist_RF)
        med_dist_RF = [0.3, 0.6, 0.6, 0.9]
        self.med_RF = self.get_membership_value(RF_distance, med_dist_RF)
        far_dist_RF = [0.6, 0.9, 1.2, 1.2]
        self.far_RF = self.get_membership_value(RF_distance, far_dist_RF)

        # Front
        near_dist_F = [0.0, 0.0, 0.3, 0.6]
        self.near_F = self.get_membership_value(F_distance, near_dist_F)
        med_dist_F = [0.3, 0.6, 0.6, 0.9]
        self.med_F = self.get_membership_value(F_distance, med_dist_F)
        far_dist_F = [0.6, 0.9, 1.2, 1.2]
        self.far_F = self.get_membership_value(F_distance, far_dist_F)

        # Left Front
        near_dist_LF = [0.0, 0.0, 0.3, 0.6]
        self.near_LF = self.get_membership_value(LF_distance, near_dist_LF)
        med_dist_LF = [0.3, 0.6, 0.6, 0.9]
        self.med_LF = self.get_membership_value(LF_distance, med_dist_LF)
        far_dist_LF = [0.6, 0.9, 1.2, 1.2]
        self.far_LF = self.get_membership_value(LF_distance, far_dist_LF)

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
        right_turning = [-0.4, -0.6, -0.8, -1.0]
        self.right = self.calculate_centroid(right_turning)
        # self.right = -0.7

        self.low_turn = 0.1

        left_turning = [0.4, 0.6, 0.8, 1.0]
        self.left = self.calculate_centroid(left_turning)
        # self.left = -0.7

        OA_rules_dict = self.set_rules_OA()

        LMS_num = []
        LMS_denom = []
        RMS_num = []
        RMS_denom = []

        # For each rule will set membership values
        for rule in OA_rules_dict.values():
            value_1 = rule[0][0]
            value_2 = rule[0][1]
            value_3 = rule[0][2]
            value_4 = rule[0][3]  # Turning COM
            value_5 = rule[0][4]  # Speed COM

            firing_strength = min(value_1, value_2, value_3)

            if firing_strength > 0:
                LMS = value_4
                RMS = value_5

                # Defuzzing / Calculate speed and steering outputs
                LMS_num.append(firing_strength*LMS)
                RMS_num.append(firing_strength*RMS)
                LMS_denom.append(firing_strength)
                RMS_denom.append(firing_strength)

                rule_fired = str(list(OA_rules_dict.keys())[
                                 list(OA_rules_dict.values()).index(rule)])
                self.get_logger().info('Rule fired: ' + str(rule_fired))
            else:
                pass  # Rule not applied

        # Motor Turning and speed
        LMS = sum(LMS_num) / sum(LMS_denom)  # Steering
        RMS = sum(RMS_num) / sum(RMS_denom)  # Speed

        return(LMS, RMS)

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

    def set_rules_OA(self):
        rules_OA = defaultdict(list)

        #                    [ RF ]        [ F ]       [ LF ]     [ Turning ] [ Speed ]
        rules_OA[0].append([self.near_RF, self.near_F, self.near_LF, self.right, self.slow])
        rules_OA[1].append([self.near_RF, self.near_F, self.med_LF, self.left, self.slow])
        rules_OA[2].append([self.near_RF, self.near_F, self.far_LF, self.left, self.slow])

        rules_OA[3].append([self.near_RF, self.med_F, self.near_LF, self.low_turn, self.slow])
        rules_OA[4].append([self.near_RF, self.med_F, self.med_LF, self.left, self.slow])
        rules_OA[5].append([self.near_RF, self.med_F, self.far_LF, self.left, self.slow])

        rules_OA[6].append([self.near_RF, self.far_F, self.near_LF, self.low_turn, self.med])
        rules_OA[7].append([self.near_RF, self.far_F, self.med_LF, self.left, self.slow])
        rules_OA[8].append([self.near_RF, self.far_F, self.far_LF, self.left, self.med])

        rules_OA[9].append([self.med_RF, self.near_F, self.near_LF, self.right, self.slow])
        rules_OA[10].append([self.med_RF, self.near_F, self.med_LF, self.right, self.slow])
        rules_OA[11].append([self.med_RF, self.near_F, self.far_LF, self.left, self.slow])

        rules_OA[12].append([self.med_RF, self.med_F, self.near_LF, self.right, self.slow])
        rules_OA[13].append([self.med_RF, self.med_F, self.med_LF, self.low_turn, self.med])
        rules_OA[14].append([self.med_RF, self.med_F, self.far_LF, self.left, self.med])

        rules_OA[15].append([self.med_RF, self.far_F, self.near_LF, self.right, self.slow])
        rules_OA[16].append([self.med_RF, self.far_F, self.med_LF, self.left, self.med])
        rules_OA[17].append([self.med_RF, self.far_F, self.far_LF, self.left, self.med])

        rules_OA[18].append([self.far_RF, self.near_F, self.near_LF, self.right, self.med])
        rules_OA[19].append([self.far_RF, self.near_F, self.med_LF, self.right, self.med])
        rules_OA[20].append([self.far_RF, self.near_F, self.far_LF, self.right, self.slow])

        rules_OA[21].append([self.far_RF, self.med_F, self.near_F, self.right, self.slow])
        rules_OA[22].append([self.far_RF, self.med_F, self.med_F, self.right, self.med])
        rules_OA[23].append([self.far_RF, self.med_F, self.far_LF, self.left, self.med])

        rules_OA[24].append([self.far_RF, self.far_F, self.near_LF, self.right, self.med])
        rules_OA[25].append([self.far_RF, self.far_F, self.med_LF, self.right, self.med])
        rules_OA[26].append([self.far_RF, self.far_F, self.far_LF, self.low_turn, self.fast])

        return rules_OA

    def set_rules_REF(self):
        rules_REF = defaultdict(list)

        #                       [ RF ]        [ RB ]     [ Turning ] [ Speed ]
        rules_REF[0].append([self.near_RF, self.near_RB, self.left, self.slow])
        rules_REF[1].append([self.near_RF, self.med_RB, self.left, self.slow])
        rules_REF[2].append([self.near_RF, self.far_RB, self.left, self.slow])

        rules_REF[3].append([self.med_RF, self.near_RB, self.right, self.slow])
        rules_REF[4].append([self.med_RF, self.med_RB, self.no_turn, self.med])
        rules_REF[5].append([self.med_RF, self.far_RB, self.left, self.med])

        rules_REF[6].append([self.far_RF, self.near_RB, self.right, self.slow])
        rules_REF[7].append([self.far_RF, self.med_RB, self.right, self.med])
        rules_REF[8].append([self.far_RF, self.far_RB, self.right, self.fast])

        return rules_REF

    def timer_callback(self):
        self.higher_fuzzification()
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
