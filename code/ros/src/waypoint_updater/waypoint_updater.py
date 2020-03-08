#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import math
import numpy as np
from scipy.spatial import KDTree


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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish.
# Selected ahead waypoints' indexes to publish (in order to save interpolations' computating time)
LOOKAHEAD_WPS_MASK = [0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 16, 20, 28, 36, 52, 68, 100, 132, 196]
MAX_DECEL = 0.5 # Max deceleration
STOPPING_WPS_BEFORE = 4 # Number of waypoints to stop before a traffic light line
FREQUENCY = 50 # 50Hz


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Adding other member variables needed
        self.pose = None
        #self.velocity = None
        self.base_waypoints = None
        self.waypoints_tree = None
        self.stop_line_wp_idx = -1

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        #rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        
        self.loop()

    def loop(self):
        rate = rospy.Rate(FREQUENCY)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints and self.waypoints_tree:
                # Getting the final waypoints
                final_waypoints = self.get_final_waypoints()
                self.publish_waypoints(final_waypoints)
            rate.sleep()

    def publish_waypoints(self, final_waypoints):
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = final_waypoints
        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        self.pose = msg



    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        self.waypoints_tree = KDTree(
            [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y]
             for waypoint in waypoints.waypoints])

    def traffic_cb(self, msg):
        # Callback for /traffic_waypoint message.
        self.stop_line_wp_idx = msg.data



    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoints_tree.query([x, y], 1)[1]

        # Checking if closest point is ahead or behind the vehicle
        closest_waypoint = self.base_waypoints.waypoints[closest_idx]
        prev_waypoint = self.base_waypoints.waypoints[
            (closest_idx - 1) if closest_idx > 0 else (len(self.base_waypoints.waypoints) - 1)]
        closest_coord = [closest_waypoint.pose.pose.position.x, closest_waypoint.pose.pose.position.y]
        prev_coord = [prev_waypoint.pose.pose.position.x, prev_waypoint.pose.pose.position.y]

        # Equation for hyperplane through closest coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.base_waypoints.waypoints)
        return closest_idx

    def get_final_waypoints(self):
        closest_idx = self.get_closest_waypoint_idx()
        # We want the car to stop at the end of the track, so not doing module
        farthest_idx = min(closest_idx + LOOKAHEAD_WPS, len(self.base_waypoints.waypoints))

        if self.stop_line_wp_idx == -1 or self.stop_line_wp_idx >= farthest_idx or self.stop_line_wp_idx < closest_idx + 1:
            # If there is no red traffic light ahead, just adding next selected waypoints
            return self.accelerate_to_target_velocity(closest_idx, farthest_idx)

        else:
            # If there is a red traffic light ahead, modifying the waypoints velocity to gradually stop
            return self.stop_at_stop_line(closest_idx, farthest_idx)

    def accelerate_to_target_velocity(self, closest_idx, farthest_idx):
        final_waypoints = []

        for i in LOOKAHEAD_WPS_MASK:
            idx = closest_idx + i
            if idx < farthest_idx:
                wp = self.base_waypoints.waypoints[idx]
                final_waypoints.append(wp)

        return final_waypoints

    def stop_at_stop_line(self, closest_idx, farthest_idx):
        final_waypoints = []
        # Index of the closest waypoint point before the stop line of the traffic light
        stop_idx = max(self.stop_line_wp_idx - STOPPING_WPS_BEFORE, closest_idx)
        target_wp = self.base_waypoints.waypoints[stop_idx]
        dist = 0.0

        for i in LOOKAHEAD_WPS_MASK[::-1]:
            # For each one of the selected waypoints (starting from the farthest one),
            # calculating the distance to the stop line and adjust the velocity in order to gradually stop
            idx = closest_idx + i
            if idx < farthest_idx:
                wp = self.base_waypoints.waypoints[idx]
                p = Waypoint()
                p.pose = wp.pose
                vel = 0.0

                if idx < stop_idx:
                    # Calculating the distance from the stop line to the current waypoint
                    dist += math.sqrt((target_wp.pose.pose.position.x - wp.pose.pose.position.x)**2 +
                                      (target_wp.pose.pose.position.y - wp.pose.pose.position.y)**2)
                    # Reducing the velocity according to the max acceleration
                    vel = math.sqrt(2 * MAX_DECEL * dist)
                    if vel < 1.0:
                        vel = 0.0

                p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
                final_waypoints.insert(0, p)

        return final_waypoints


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
