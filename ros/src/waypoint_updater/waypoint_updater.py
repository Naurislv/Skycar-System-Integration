#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

def get_closest_waypoint(pose_x, pose_y, waypoints):

    # initial variables
    closest_distance = 100000.0
    closest_point = 0

    for i in range(len(waypoints)):
        # extract waypoint x,y
        wp_x = waypoints[i].pose.pose.position.x
        wp_y = waypoints[i].pose.pose.position.y
        # compute distance from car x,y
        distance = math.sqrt((wp_x - pose_x) ** 2 + (wp_y - pose_y) ** 2)
        # is this point closer than others found so far
        if distance < closest_distance:
            closest_distance = distance
            closest_point = i

    # return closest point found
    return closest_point


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        rospy.loginfo('Init waypoint_updater')

        self.current_pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        # Format of self.var = init_value - declare and initialise

        # Will need a list of waypoints
        self.waypoints = []

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement

        # msg will be a geometry_msgs/PoseStamped message
        # extract the current car x, y
        pose_x = msg.pose.position.x
        pose_y = msg.pose.position.y
        # find the closest waypoint
        closest_waypoint = get_closest_waypoint(pose_x, pose_y, self.waypoints)

        # get waypoints ahead of the car
        # this currently sends as many as are available
        # should this fail if there aren't enough waypoints and just wait until there area enough? - NM
        waypoints_ahead = []
        n_waypoints = len(self.waypoints)               # can only get this many waypoints
        if n_waypoints > LOOKAHEAD_WPS:
            n_waypoints = LOOKAHEAD_WPS                 # max waypoints to pass over
        for i in range(n_waypoints):
            waypoints_ahead.append(self.waypoints[closest_waypoint+i])

        # structure the data to match the expected styx_msgs/Lane form
        lane = Lane()
        lane.waypoints = waypoints_ahead                # list of waypoints ahead of the car
        lane.header.stamp = rospy.Time(0)               # timestamp
        lane.header.frame_id = msg.header.frame_id      # matching up with the input message frame_id

        # publish the waypoints list
        self.final_waypoints_pub.publish(lane)

        #pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement

        # receive waypoints in message type styx_msgs/Lane form

        # save these, to use when pose_cb is called later
        self.waypoints = waypoints.waypoints

        # we only need the message once, unsubscribe as soon as we handled the message
        self.base_waypoints_sub.unregister()

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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
