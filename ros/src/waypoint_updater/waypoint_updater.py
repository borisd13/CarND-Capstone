#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Quaternion
from styx_msgs.msg import Lane, Waypoint

import numpy as np
import math
import tf
import time
from functools import wraps

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 40 # Number of waypoints we will publish. You can change this number
LOOKAHEAD_WPS_DIST = 40.0
RAD2DGREES = 57.32
MAXVELOCITY = 20 * 0.44 # 20kmph to mph
DEG = 3


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # rospy.Subscriber('/obstacle_waypoints', , )
        # rospy.Subscriber('/traffic_waypoints', , )

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.cte_pub = rospy.Publisher('cross_track_error', Float64, queue_size=1)

        # TODO: Add other member variables you need below
        self.basewaypoints = None
        self.basewaypoint_xs = None
        self.basewaypoint_ys = None
        self.current_position = None
        self.yaw = None
        self.closest_wp_index = None

        # rospy.spin()

        self.loop()

    def loop(self):

        rate = rospy.Rate(50)

        while self.basewaypoints is None or self.current_position is None:

            rate.sleep()

        while not rospy.is_shutdown():

            current_time = time.time()

            pose_time = time.time()
            self.yaw = self.get_current_car_direction()
            rospy.loginfo("duration get_yaw:{}".format(time.time() - pose_time))

            pose_time = time.time()
            self.closest_wp_index = self.get_closest_waypoint_index()
            rospy.loginfo("duration get_closest_waypoint_index:{}".format(time.time() - pose_time))

            pose_time = time.time()
            smooth_waypoints  = self.get_smooth_waypoints_coordinates(self.closest_wp_index, LOOKAHEAD_WPS_DIST)
            rospy.loginfo("duration get_smooth_waypoints_cord:{}".format(time.time() - pose_time))

            pose_time = time.time()
            final_waypoints = self.get_waypoints(smooth_waypoints)
            rospy.loginfo("duration get_waypoints:{}".format(time.time() - pose_time))

            pose_time = time.time()
            self.final_waypoints_pub.publish(final_waypoints)
            rospy.loginfo("duration final_wp publish:{}".format(time.time() - pose_time))

            pose_time = time.time()
            cte = self.get_cte(smooth_waypoints)
            rospy.loginfo("duration get_cte:{}".format(time.time() - pose_time))

            pose_time = time.time()
            self.cte_pub.publish(cte)
            rospy.loginfo("duration publish cte:{}".format(time.time() - pose_time))

            # rospy.loginfo("yaw:{}, cte:{}, s_wp_len:{}".format(self.yaw*180/3.14, cte, len(smooth_waypoints)))
            rospy.loginfo("1 iter duration time:{}\n".format(time.time() - current_time))

            rate.sleep()

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_position = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.basewaypoints = waypoints.waypoints
        self.basewaypoint_xs = np.array([wp.pose.pose.position.x for wp in self.basewaypoints])
        self.basewaypoint_ys = np.array([wp.pose.pose.position.y for wp in self.basewaypoints])

    def get_waypoints(self, smooth_waypoints):

        final_waypoints = Lane()
        final_waypoints.header.frame_id = '/world'
        final_waypoints.header.stamp = rospy.Time.now()

        for wp in smooth_waypoints:
             final_waypoint = wp
             final_waypoint.twist.twist.linear.x = MAXVELOCITY
             final_waypoints.waypoints.append(final_waypoint)

        return final_waypoints

    def get_smooth_waypoints_coordinates(self, start_index, wps_distance):

        # backward_wp = self.decimating_waypoints(start_index-1, 1.0, wps_distance, False)
        forward_wp = self.get_sparsed_waypoints(start_index, 1.0, wps_distance, True)
        # forward_wp = self.decimating_waypoints(start_index, 0.3, wps_distance, True)

        # the case of backward waypoints
        # poly = self.get_polynomial(backward_wp + forward_wp)
        # poly = self.get_polynomial(forward_wp)

        # wp_xs = [wp.pose.pose.position.x for wp in forward_wp]
        # new_wp_ys = [np.polyval(poly, x) for x in wp_xs]
        # for i in range(len(forward_wp)):
        #     # forward_wp[i].pose.pose.position.x = wp_xs[i]
        #     forward_wp[i].pose.pose.position.y = new_wp_ys[i]

        return forward_wp

    def get_sparsed_waypoints(self, start_index, min_distances, total_distance, forward=True):
        """
        Create a list of sparse waypoints starting from start_index. The distance between
        each waypoints is setted larger than min_distances
        :param start_index: int
        :param min_distances: float
        :param total_distance: float representing distance to farthest waypoint
        :param forward: bool. if forward is True, create a list of waypoints towards the ahead of vehicle.
        :return: list of waypoints
        Note: This function is necessary to remove the noise included in the waypoints.
        """

        waypoints = []
        index = start_index
        base_wp_index = 0
        distance = 0

        while distance <= total_distance and len(waypoints) < 20:

            if len(waypoints) == 0:
                waypoints.append(self.basewaypoints[index])
                base_wp_index = index

                # translated_base_x, _ = self.get_translated_waypoint(waypoints[-1])
                # distance += translated_base_x

                translated_current_x, _ = self.get_frenet_coordinate(self.current_position)
                translated_next_x, _ = self.get_frenet_coordinate(waypoints[-1].pose)
                distance += abs(translated_next_x - translated_current_x)

                # logging
                # rospy.loginfo("base_idx:{}, wps_dist:{}".format(base_wp_index, distance))
                # rospy.loginfo("base_idx:{}, base_x:{}, base_y:{}, wps_dist:{}".format(
                #     base_wp_index, self.basewaypoints[base_wp_index].pose.pose.position.x,
                #     self.basewaypoints[base_wp_index].pose.pose.position.y, distance))

            else:
                base = self.basewaypoints[base_wp_index]
                target = self.basewaypoints[index]

                # rotate wps
                # translated_base_x, _ = self.get_translated_corrdinates(base)
                # translated_target_x, _ = self.get_translated_corrdinates(target)

                # convert frenet
                translated_base_x, _ = self.get_frenet_coordinate(base.pose)
                translated_target_x, _ = self.get_frenet_coordinate(target.pose)

                wps_distance = abs(translated_target_x - translated_base_x)

                if wps_distance > min_distances:
                    waypoints.append(target)

                    # for logging
                    # base_x = base.pose.pose.position.x
                    # base_y = base.pose.pose.position.y
                    # target_x = target.pose.pose.position.x
                    # target_y = target.pose.pose.position.y

                    # rospy.loginfo("base_idx:{}, base_x:{}, base_y:{}, target_idx:{}, target_x:{}, target_y:{}, tr_dist:{:.3f}, orgn_dist:{:.3f}".format(
                    # base_wp_index, base_x, base_y, index, target_x, target_y, wps_distance, target_x - base_x))

                    base_wp_index = index

                    distance += wps_distance

            if forward:
                index += 1

            else:
                index -= 1

            index %= len(self.basewaypoints)

        return waypoints

    def get_closest_waypoint_index(self):
        """
        Return the index of waypoint closest to current vehicle posistion.
        :return: int representing closest waypoint index
        """

        diff_xs = self.basewaypoint_xs - self.current_position.pose.position.x
        diff_ys = self.basewaypoint_ys - self.current_position.pose.position.y

        distances = [math.sqrt(x**2 + y**2) for x, y in zip(diff_xs, diff_ys)]

        closest_waypoint_index = np.argmin(distances)

        while self.is_forward(closest_waypoint_index):

            closest_waypoint_index += 1
            closest_waypoint_index %= len(self.basewaypoints)

        return closest_waypoint_index

    def is_forward(self, wp_index):
        """
        Check if the given waypoint index is in ahead of the car.
        If the given waypoint index is in ahead of the car, return True.
        :param wp_index: int representing waypoint index
        :return: bool
        """

        waypoint = self.basewaypoints[wp_index]
        current_frenet_s, current_frenet_d = self.get_frenet_coordinate(self.current_position)
        target_frenet_s, target_frenet_d = self.get_frenet_coordinate(waypoint.pose)

        is_forward = target_frenet_s - current_frenet_s

        # for logging
        # rospy.loginfo("target index:{}, current_frenet_s:{}, target_frenet_d:{}, diff:{}".format(
        #     wp_index, current_frenet_s, target_frenet_s, is_forward))

        if is_forward > 0:
            return True

        else:
            return False

    def get_cte(self, waypoints):

        translated_ptsx, translated_ptsy = self.get_translated_corrdinates(waypoints)

        polynomial = np.polyfit(translated_ptsx, translated_ptsy, DEG)
        cte = np.polyval(polynomial, 0)

        return cte

    def get_translated_corrdinates(self, waypoints):
        """
        Given the waypoints, convert global coordinates waypoints to vehicle's coordinates,
        :param waypoints:
        :return: vehicle's coordinates
        """

        current_x = self.current_position.pose.position.x
        current_y = self.current_position.pose.position.y

        translated_ptsx = []
        translated_ptsy = []

        for wp in waypoints:

            ptsx = wp.pose.pose.position.x - current_x
            ptsy = wp.pose.pose.position.y - current_y

            translated_ptsx.append(ptsx*math.cos(-self.yaw) - ptsy*math.sin(-self.yaw))
            translated_ptsy.append(ptsx*math.sin(-self.yaw) + ptsy*math.cos(-self.yaw))

        return translated_ptsx, translated_ptsy

    def get_current_car_direction(self):

        quaternion = (self.current_position.pose.orientation.x,
                      self.current_position.pose.orientation.y,
                      self.current_position.pose.orientation.z,
                      self.current_position.pose.orientation.w)

        euler = tf.transformations.euler_from_quaternion(quaternion)

        return euler[2]

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def get_frenet_coordinate(self, waypoint):
        """
        Transform from waypoint x,y coordinates to Frenet s,d coordinates
        :param waypoint: waypoint
        :return: two float values representing s and d coordinates
        """

        next_wp_index = self.get_next_wp_index(waypoint)
        prev_wp_index = next_wp_index - 1

        if next_wp_index == 0:
            prev_wp_index = len(self.basewaypoint_xs) - 1

        prev_wp_x = self.basewaypoint_xs[prev_wp_index]
        prev_wp_y = self.basewaypoint_ys[prev_wp_index]
        nx = self.basewaypoint_xs[next_wp_index] - prev_wp_x
        ny = self.basewaypoint_ys[next_wp_index] - prev_wp_y

        xx = waypoint.pose.position.x - prev_wp_x
        xy = waypoint.pose.position.y - prev_wp_y

        # find the projection of x onto n
        proj_norm = (xx*nx + xy*ny) / (nx*nx + ny*ny)
        proj_x = proj_norm*nx
        proj_y = proj_norm*ny

        frenet_d = get_distance(xx, xy, proj_x, proj_y)

        center_x = 1000 - prev_wp_x
        center_y = 2000 - prev_wp_y
        center_to_pos = get_distance(center_x, center_y, xx, xy)
        center_to_ref = get_distance(center_x, center_y, proj_x, proj_y)

        if center_to_pos <= center_to_ref:
            frenet_d *= -1

        frenet_s = 0
        index = 0

        while index < prev_wp_index:

            wp_x = self.basewaypoint_xs[index]
            wp_y = self.basewaypoint_ys[index]

            next_wp_x = self.basewaypoint_xs[index+1]
            next_wp_y = self.basewaypoint_ys[index+1]

            frenet_s += get_distance(wp_x, wp_y, next_wp_x, next_wp_y)
            index += 1

        frenet_s += get_distance(0, 0, proj_x, proj_y)

        return frenet_s, frenet_d


    def get_next_wp_index(self, target_wp):
        """
        Given target waypoint, return the index of waypoint closest to given waypoint.
        :param target_wp: waypoint object
        :return: index representing closest waypoint
        """

        diff_xs = self.basewaypoint_xs - target_wp.pose.position.x
        diff_ys = self.basewaypoint_ys - target_wp.pose.position.y

        distances = [math.sqrt(x**2 + y**2) for x, y in zip(diff_xs, diff_ys)]
        closest_wp_index = np.argmin(distances)

        closest_wp = self.basewaypoints[closest_wp_index]
        diff_x = self.current_position.pose.position.x - closest_wp.pose.pose.position.x
        diff_y = self.current_position.pose.position.y - closest_wp.pose.pose.position.y

        heading = math.atan2(diff_y, diff_x)
        angle = abs(self.yaw - heading)

        if angle > (math.pi/4):
            closest_wp_index += 1

        return closest_wp_index

    def get_distance_to_waypoints(self, waypoint1, waypoint2):
        """
        Given two waypoints, compute distance of these waypoints
        """

        wp1_coordinate = waypoint1.pose.position
        wp2_coordinate = waypoint2.pose.position

        return get_distance(wp1_coordinate.x, wp1_coordinate.y, wp2_coordinate.x, wp2_coordinate.y)

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


def get_distance(x1, y1, x2, y2):

    diff_x = x2 - x1
    diff_y = y2 - y1

    return math.sqrt(diff_x**2 + diff_y**2)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')