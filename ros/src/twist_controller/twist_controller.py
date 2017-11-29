from pid import PID
from yaw_controller import YawController
from lowpass import LowPassFilter
import rospy
import tf
import time

import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704 # miles per hour to meters per second
MIN_SPEED = 0
STEER_MAX = 1.0


class Controller(object):
    def __init__(self, kp, ki, kd, min, max, wheel_base, steer_ratio, min_speed,
                 max_lat_accel, max_steer_angle, vehicle_weight, brake_deadband, wheel_radius):
        # TODO: Implement
        # self.throttle_pid = PID(kp, ki, kd, min, max)
        self.throttle_pid = PID(kp, ki, kd, min, max)
        # self.steer_pid = PID(2.1, 0.00001, 0.0001, min, max)
        self.steer_pid = PID(0.2, 0.0005, 0.15, -STEER_MAX, STEER_MAX)
        self.brake_pid = PID(2.0, 0.01, 0.01, min, max)
        # self.low_pass_filter = LowPassFilter(0.2, 0.1)

        self.yaw_controller = YawController(wheel_base, steer_ratio,
                                            min_speed, max_lat_accel, max_steer_angle)

        # self.lowpass_filter = LowPassFilter()
        self.vehicle_weight = vehicle_weight
        self.brake_deadband = brake_deadband
        self.wheel_radius = wheel_radius
        self.prev_time = None
        # self.to_mps = 0.00028

    def control(self, twist_cmd, current_velocity, dbw_status, current_cte):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if dbw_status.data:

            # rospy.loginfo("proposed x:{}, current x:{}, error:{}".format(
            #     twist_cmd.twist.linear.x, current_velocity.twist.linear.x, error))

            # rospy.loginfo("twist_cmd linear, x:{}, y:{}, z:{}".format(
            #     twist_cmd.twist.linear.x, twist_cmd.twist.linear.y, twist_cmd.twist.linear.z))

            # rospy.loginfo("twist_cmd angular, x:{}, y:{}, z:{}".format(
            #     twist_cmd.twist.angular.x, twist_cmd.twist.angular.y, twist_cmd.twist.angular.z))

            # rospy.loginfo("current vel linear, x:{}, y:{}, z:{}".format(
            #     current_velocity.twist.linear.x, current_velocity.twist.linear.y, current_velocity.twist.linear.z))

            # rospy.loginfo("current vel angular, x:{}, y:{}, z:{}".format(
            #     current_velocity.twist.angular.x, current_velocity.twist.angular.y, current_velocity.twist.angular.z))

            if self.prev_time is None:
                current_time = time.time()
                dt = 0.2

            else:
                current_time = time.time()
                dt = current_time - self.prev_time

            # target_linear_x = max(MIN_SPEED, twist_cmd.twist.linear.x)
            current_speed = math.sqrt(current_velocity.twist.linear.x**2 + current_velocity.twist.linear.y**2)
            target_speed = math.sqrt(twist_cmd.twist.linear.x**2 + twist_cmd.twist.linear.y**2)

            diff_linear_x = target_speed - current_speed
            rospy.loginfo("target_speed:{}, current_speed:{}, dt:{}".format(target_speed, current_speed, dt))

            if diff_linear_x <= 0:
                throttle = 0
                brake = self.get_brake_value(twist_cmd, current_velocity)
                brake = self.brake_pid.step(brake, dt)

            elif current_velocity.twist.linear.x > 8.8:
                throttle = 0
                brake = self.brake_pid.step(-5, dt)

            else:
                throttle = self.throttle_pid.step(diff_linear_x, dt)
                brake = 0

            # steer_error = self.yaw_controller.get_steering(twist_cmd.twist.linear.x,
            #                                          twist_cmd.twist.angular.z,
            #                                          current_velocity.twist.linear.x)

            # steer = self.steer_pid.step(steer_error, dt)
            steer = self.steer_pid.step(current_cte, dt)
            # steer = self.low_pass_filter.filter(steer)
            rospy.loginfo("current cte:{}".format(current_cte))

            self.prev_time = current_time

            rospy.loginfo("throttle:{}, brake:{}, steer:{}\n".format(throttle, brake, steer))
            cmd = throttle, brake, steer

        else:
            cmd = 0., 0., 0.

        return cmd

    def get_brake_value(self, twist_cmd, current_velocity):

        diff_x = current_velocity.twist.linear.x - twist_cmd.twist.linear.x
        # diff_x = twist_cmd.twist.linear.x - current_velocity.twist.linear.x
        delta_acc = diff_x

        f = self.vehicle_weight * delta_acc
        torque = f * self.wheel_radius

        if torque < self.brake_deadband:
            torque = 0

        return torque / 4.0

def get_distance(x, y, z):

    return math.sqrt(x**2 + y**2 + z**2)


def get_current_car_direction(orientation):
    quaternion = (orientation.x,
                  orientation.y,
                  orientation.z,
                  orientation.w)

    euler = tf.transformations.euler_from_quaternion(quaternion)

    return euler[2]
