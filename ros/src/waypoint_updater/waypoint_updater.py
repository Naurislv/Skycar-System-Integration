#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32, Bool

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

STOP_DISTANCE = 100     # Distance to traffic lights within which we may stop the car
STOP_LINE_OFFSET = 10   # Distance back from lights to actually stop the car

REFERENCE_VELOCITY = 11.0   # Reference velocity when restarting the car
REFERENCE_DISTANCE = 30     # Distance to get back up to reference velocity

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
        self.traffic_waypoint_sub = rospy.Subscriber('/traffic_waypoint', Int32 ,self.traffic_cb)
        self.dbw_enabled_sub = rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        self.current_velocity = rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        # Format of self.var = init_value - declare and initialise
        self.closest_waypoint = -1
        self.dbw_enabled = False
        self.current_velocity = 0.0

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
        self.closest_waypoint = get_closest_waypoint(pose_x, pose_y, self.waypoints)

        # get waypoints ahead of the car
        # this currently sends as many as are available
        # should this fail if there aren't enough waypoints and just wait until there area enough? - NM
        waypoints_ahead = []
        n_waypoints = len(self.waypoints)               # can only get this many waypoints
        if n_waypoints > LOOKAHEAD_WPS:
            n_waypoints = LOOKAHEAD_WPS                 # max waypoints to pass over
        for i in range(n_waypoints):
            waypoints_ahead.append(self.waypoints[self.closest_waypoint+i])

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

        # Test code - echo out the traffic light waypoint if it is non-zero
        #if msg > 0:
        #    rospy.loginfo("[test] traffic_cb next red light waypoint #%s", msg)

        next_red_light = msg.data       # get the waypoint ref of the next red light (-1 if none)

        if (self.closest_waypoint > 0) and (self.dbw_enabled):   # skip if no position or manual driving

            # get the nearest waypoint velocity
            start_point_velocity = self.get_waypoint_velocity(self.waypoints[self.closest_waypoint])

            if next_red_light > 0:
                #rospy.loginfo("[test] traffic_cb next red light waypoint: %s", next_red_light)
                # now check if the red light is close enough that we need to worry about it...
                # self.closest_waypoint will have the current car position (updated in pose_cb)
                distance_to_red = self.distance(self.waypoints, self.closest_waypoint, next_red_light)
                #rospy.loginfo("[test] distance to red light: %s", distance_to_red)
                #rospy.loginfo("[test] current waypoint velocity = " + str(start_point_velocity))
                if distance_to_red < STOP_DISTANCE:
                    rospy.loginfo("[test] stopping...")
                    # smoothly stop over the waypoints up to next_red_light waypoint
                    for i in range(self.closest_waypoint, next_red_light + 1):
                        # get the distance to the i-th way point
                        i_point_distance = self.distance(self.waypoints, self.closest_waypoint, i)
                        #rospy.loginfo("[test] distance to point " + str(i) + " = " + str(i_point_distance))
                        if (distance_to_red - STOP_LINE_OFFSET) > 0:
                            i_point_target_velocity = (i_point_distance / (distance_to_red - STOP_LINE_OFFSET )) * start_point_velocity * -1
                            i_point_target_velocity += start_point_velocity
                            if i_point_target_velocity < 0.0:
                                #rospy.loginfo("[test] error - negative velocity... v=" + str(i_point_target_velocity))
                                pass
                        else:
                            i_point_target_velocity = 0.0
                        self.set_waypoint_velocity(self.waypoints, i, i_point_target_velocity)

            else:
                rospy.loginfo("[test] traffic_cb next light is green, current velocity: %s", self.current_velocity)
                # don't want to stop, but do want to make sure we keep going...
                if start_point_velocity < REFERENCE_VELOCITY:
                    # speed up again
                    rospy.loginfo("[test] speeding up again...")
                    #rospy.loginfo("[test] closest waypoint: %s", self.closest_waypoint)
                    # smooth acceleration over the planned distance
                    for i in range(self.closest_waypoint, self.closest_waypoint + LOOKAHEAD_WPS):
                        #rospy.loginfo("[test] updating waypoint: %s ", i)
                        i_point_distance = self.distance(self.waypoints, self.closest_waypoint, i)
                        if i_point_distance < REFERENCE_DISTANCE:
                            i_point_target_velocity = REFERENCE_VELOCITY * (REFERENCE_DISTANCE - i_point_distance)
                            i_point_target_velocity = i_point_target_velocity / REFERENCE_DISTANCE
                            self.set_waypoint_velocity(self.waypoints, i, i_point_target_velocity)
                            #rospy.loginfo("[test] updating waypoint: " +str(i) + " to velocity " +str(i_point_target_velocity))
                        else:
                            #self.set_waypoint_velocity(self.waypoints, i, REFERENCE_VELOCITY)
                            #rospy.loginfo("[test] defaulting waypoint: " + str(i) + " to reference velocity")
                            break

        #pass

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

    def dbw_enabled_cb(self, msg):
        # check if drive-by-wire is enabled (i.e. the car is not in manual mode)
        self.dbw_enabled = msg.data

    def current_velocity_cb(self, msg):
        # store the current velocity TwistStamped message
        self.current_velocity = msg.twist.linear.x

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
