#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Float64

import math
import copy
import tf.transformations   # to get Euler coordinates
import numpy as np

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

LOOKAHEAD_WPS = 30 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        self.ego_pos = None
        self.wps = None
        self.final_wps = None
        self.first_pass = True
        self.wps_len = None
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)
        self.cte_pub = rospy.Publisher('/cross_track_error', Float64, queue_size=1)
        self.loop()

    def loop(self):

        rate = rospy.Rate(10)

        while not all([self.wps, self.ego_pos]):

            rate.sleep()

        while not rospy.is_shutdown() and all([self.wps, self.ego_pos]):

            if self.wps is not None:	#Don't proceed until we have received waypoints

                # Get car orientation
                car_x, car_y = self.ego_pos.position.x, self.ego_pos.position.y
                quaternion = (self.ego_pos.orientation.x, self.ego_pos.orientation.y,
                              self.ego_pos.orientation.z, self.ego_pos.orientation.w)
                euler = tf.transformations.euler_from_quaternion(quaternion)
                car_yaw = euler[2]

                #return the index of the closest waypoint ahead of us
                closest_idx_waypoint = self.closest_waypoint_ahead(car_x, car_y, car_yaw, self.wps.waypoints)
                # get sparse waypoints indices
                waypoint_idx_list = self.get_sparse_waypoint_indices(closest_idx_waypoint, LOOKAHEAD_WPS, 1.0, car_yaw)

                #final waypoints is a subset of original set of waypoints
                self.final_wps.waypoints = [self.wps.waypoints[index] for index in waypoint_idx_list]

                #check we didn't reach the end of the list and otherwise loopback to start of the list
                if len(self.final_wps.waypoints) < LOOKAHEAD_WPS:
                    extra_points_needed = LOOKAHEAD_WPS - len(self.final_wps.waypoints)

                    # we need to get points from the start of the list ensuring next point is closest ahead
                    last_x = self.wps.waypoints[-1].pose.pose.position.x
                    last_y = self.wps.waypoints[-1].pose.pose.position.y
                    last_x2 = self.wps.waypoints[-2].pose.pose.position.x
                    last_y2 = self.wps.waypoints[-2].pose.pose.position.y
                    last_yaw = math.atan2(last_y - last_y2, last_x - last_x2)
                    # we don't include last points of the list to ensure we go back to the beginning of the list
                    first_extra_point = self.closest_waypoint_ahead(last_x, last_y, last_yaw, self.wps.waypoints[0:-10])

                    #we complete our list to desired number of points
                    self.final_wps.waypoints.extend(self.wps.waypoints[first_extra_point:first_extra_point+extra_points_needed])

                if len(self.final_wps.waypoints) != LOOKAHEAD_WPS:
                    rospy.logwarn("List of /final_waypoints does not contain target number of elements")

                self.final_waypoints_pub.publish(self.final_wps)

                current_cte = self.get_cte(self.final_wps.waypoints, car_yaw)
                self.cte_pub.publish(current_cte)

            rate.sleep()

    def pose_cb(self, msg):
        self.ego_pos = msg.pose

    def waypoints_cb(self, waypoints):
        # Ensure we only get initial full list of waypoints as simulator keeps publishing
        # with patial list aftewards
        if self.wps is None:
            # We need to get a full copy as otherwise we just get a reference
            self.wps = copy.copy(waypoints)
            self.final_wps = copy.copy(waypoints)
            self.wps_len = len(self.wps.waypoints)

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

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def closest_waypoint_ahead(self, pos_x, pos_y, yaw, waypoints):
        ''' Return index of closest point ahead '''

        # Create some logging info
        loginfo = 'yaw: {} | x: {} | y: {}'.format(yaw, pos_x, pos_y)

        # Define unit vector for car orientation in global (x, y) coordinates
        orient_x, orient_y = math.cos(yaw), math.sin(yaw)

        # Filter waypoints to keep only the ones ahead of us by checking scalar product
        waypoints_ahead = [(n, wp) for (n, wp) in enumerate(waypoints)
                           if (orient_x * (wp.pose.pose.position.x - pos_x) +
                           orient_y * (wp.pose.pose.position.y - pos_y)) > 0]
        if not len(waypoints_ahead):
            rospy.logwarn("No points detected ahead of us")
        
        # Extract closest waypoint
        closest_waypoint = min(waypoints_ahead,
                               key = lambda wpidx: (wpidx[1].pose.pose.position.x - self.ego_pos.position.x) ** 2
                               + (wpidx[1].pose.pose.position.y - self.ego_pos.position.y) ** 2)
        closest_index = closest_waypoint[0]
        loginfo += '| Closest waypoint index: {}'.format(closest_index)
        rospy.loginfo_throttle(1, loginfo)

        return closest_index

    def get_cte(self, waypoints, yaw):

        # convert global coordinates waypoints to vehicle's coordinates
        diff_x = [wp.pose.pose.position.x - self.ego_pos.position.x for wp in waypoints]
        diff_y = [wp.pose.pose.position.y - self.ego_pos.position.y for wp in waypoints]
        translated_x = [x*math.cos(-yaw) - y*math.sin(-yaw) for x, y in zip(diff_x, diff_y)]
        translated_y = [x*math.sin(-yaw) + y*math.cos(-yaw) for x, y in zip(diff_x, diff_y)]

        polynomial = np.polyfit(translated_x, translated_y, 3)

        return np.polyval(polynomial, 0)

    def get_sparse_waypoint_indices(self, closest_waypoint_index, n_waypoints, min_distance, yaw):
        """
        Create a list of sparse waypoints starting from closest_waypoint_index.
        The distance between each waypoints is set larger than min_distance.
        """

        waypoint_indices = [closest_waypoint_index]
        base_index = closest_waypoint_index
        target_index = closest_waypoint_index + 1

        # If target_index is out of wps size, it is set zero.
        if self.wps_len <= target_index:
            target_index = 0

        while len(waypoint_indices) < n_waypoints:

            base = self.wps.waypoints[base_index].pose.pose.position
            target = self.wps.waypoints[target_index].pose.pose.position
            diff_x = target.x - base.x
            diff_y = target.y - base.y

            # Define unit vector for car orientation in global (x, y) coordinates
            orient_x, orient_y = math.cos(yaw), math.sin(yaw)
            is_ahead = (orient_x * diff_x + orient_y * diff_y) > 0

            distance = math.sqrt(diff_x**2 + diff_y**2)

            if distance >= min_distance and is_ahead:

                waypoint_indices.append(target_index)
                base_index = target_index

            target_index += 1

            # If target_index is out of wps size, it is set zero.
            if self.wps_len <= target_index:
                target_index = 0

        return waypoint_indices


if __name__ == '__main__':
    try:
	WaypointUpdater()	
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
