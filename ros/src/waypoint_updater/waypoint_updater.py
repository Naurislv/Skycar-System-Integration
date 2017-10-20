#!/usr/bin/env python
"""
    Waypoint updater node
    Controls waypoints through which the car will drive
    Also responsible for handling upcoming traffic lights
"""

# Standard imports
import math
import numpy as np
# ROS imports
import rospy
import tf
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane              #, Waypoint
from std_msgs.msg import Int32, Bool



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

LOOKAHEAD_WPS = 50          # Number of waypoints we will publish. You can change this number.  Was 200

STOP_LINE_OFFSET = 3.5      # Distance back from stop line to actually stop the car, was 28.5 for light gantry
MIN_STOP_DISTANCE = 0       # If within this distance, don't stop (already in intersection), was 28.5 for tl

REFERENCE_VELOCITY = 4.5   # Reference velocity when restarting the car
                            # 4.47m/s = 10mph
                            # 11.0m/s = ~25mph

COMFORTABLE_DECEL = 2.0     # A comfortable rate to decelerate
MAXIMUM_DECEL = 10.0        # Maximum rate to decelerate
MIN_EMERGENCY_VELOCITY = 5  # Min speed, over which will keep going at late red light, was 1.5
                            # If going slower, will do an emergency stop so as not to go through intersection

# Control states
CONTROL_STATE_UNKNOWN = -1
CONTROL_STATE_DRIVING = 1
CONTROL_STATE_STOPPING = 2

def get_closest_waypoint(pose_x, pose_y, theta, waypoints):
    """
        Get the closest waypoint for a given position

    :param pose_x:
    :param pose_y:
    :param theta:
    :param waypoints:
    :return:
    """
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

    x = waypoints[closest_point].pose.pose.position.x
    y = waypoints[closest_point].pose.pose.position.y
    heading = np.arctan2((y-pose_y), (x-pose_x))
    angle = np.abs(theta-heading)
    if angle > np.pi/4.:
        closest_point += 1
        if closest_point >= len(waypoints):
            closest_point = 0

    # return closest point found
    return closest_point

class WaypointUpdater(object):
    """
    Waypoint updater class
    Receives positions and base waypoints
    Plans future waypoint velocities in response to traffic lights
    """

    def __init__(self):
        """
        Initialise class object
        """
        rospy.init_node('waypoint_updater')
        rospy.loginfo('Init waypoint_updater')

        self.current_pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.traffic_waypoint_sub = rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # Need to know if we are driving autonomously
        self.dbw_enabled_sub = rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        # Need to know the current velocity of the car
        self.current_velocity = rospy.Subscriber('/current_velocity', TwistStamped,
                                                 self.current_velocity_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Add other member variables you need below
        self.closest_waypoint = -1                      # closest waypoint to car
        self.next_stop_line = -1                        # waypoint ID of stop line for a red light
        self.dbw_enabled = False                        # Is drive-by-wire enabled (vs manual driving)
        self.current_velocity = 0.0                     # Car current velocity
        self.control_state = CONTROL_STATE_UNKNOWN      # Car control state / finite state machine
        self.pose_x = -1.0                              # Car position x
        self.pose_y = -1.0                              # Car position y
        self.theta  = 0.0                               # Car heading angle
        # Will need a list of waypoints
        self.waypoints = []                             # List of waypoints for track

        self.sampling_rate = 10.0                       # Rate for the main loop - was 50Hz originally
        self.loop()

    def loop(self):
        """
            Loop function to publish waypoints for car to follow
        :return:
        """
        rate = rospy.Rate(self.sampling_rate)       # Was 50Hz
        while not rospy.is_shutdown():

            # find the closest waypoint, checking that we've had a position update already
            if self.pose_x > -1.0 and self.pose_y > -1.0:
                self.closest_waypoint = get_closest_waypoint(self.pose_x, self.pose_y, self.theta, self.waypoints)

            if (self.closest_waypoint > 0) and (self.dbw_enabled):
                # skip if no position or manual driving

                # get the nearest waypoint velocity
                start_point_velocity = self.get_waypoint_velocity(self.waypoints[self.closest_waypoint])

                """
                    Get the state for the car
                    Might be one of:
                        Driving - keep going, targeting the reference velocity
                        Stopping - slowing down for a red light

                    Note: will swap to *STOPPING* state when the distance to the red light is within the
                    distance from which the car can stop at a 'comfortable' rate.

                    Equations:
                        If travelling at velocity V, will cover distance d = Vt in time t
                        If travelling at velocity V, will take time t = V/C to stop,
                            where C = constant rate of deceleration
                        Hence t = d/V < V/C to be able to stop

                        To stop when distance (d) to red light becomes equal to or less than
                        the approximate distance we need in order to be able to stop
                        'comfortably', requires:

                        d < V^2 / C

                """
                if self.next_stop_line > 0:
                    # red light ahead, near or far
                    distance_to_stop_line = self.distance(self.waypoints, self.closest_waypoint,
                                                    self.next_stop_line)
                    comfort_stopping_distance = (self.current_velocity * self.current_velocity)
                    comfort_stopping_distance = comfort_stopping_distance / COMFORTABLE_DECEL
                    minimum_stop_distance = self.current_velocity * self.current_velocity
                    minimum_stop_distance = minimum_stop_distance / MAXIMUM_DECEL
                    if (distance_to_stop_line - STOP_LINE_OFFSET) < comfort_stopping_distance:
                        if self.control_state == CONTROL_STATE_DRIVING and \
                                        (distance_to_stop_line - STOP_LINE_OFFSET) < minimum_stop_distance:
                            # keep going, or will stop within intersection?
                            if self.current_velocity < MIN_EMERGENCY_VELOCITY and \
                                            distance_to_stop_line > STOP_LINE_OFFSET:
                                # can stop
                                rospy.loginfo("[test] Emergency stop case")
                                self.control_state = CONTROL_STATE_STOPPING
                            else:
                                rospy.loginfo("[test] Ignoring late red light")
                                self.control_state = CONTROL_STATE_DRIVING
                        else:
                            # should slow down and stop now
                            if self.control_state != CONTROL_STATE_STOPPING:
                                rospy.loginfo("[test] Changing to *STOPPING* state")
                            self.control_state = CONTROL_STATE_STOPPING
                    else:
                        if self.control_state == CONTROL_STATE_STOPPING:
                            # we are already stopping, stay in this state if the light is still red
                            rospy.loginfo("[test] Holding *STOPPING* state")
                        else:
                            if self.control_state != CONTROL_STATE_DRIVING:
                                rospy.loginfo("[test] Changing to *DRIVING* state")
                            self.control_state = CONTROL_STATE_DRIVING
                else:
                    # no red light
                    if self.control_state != CONTROL_STATE_DRIVING:
                        rospy.loginfo("[test] Changing to *DRIVING* state")
                    self.control_state = CONTROL_STATE_DRIVING

                # if distance_to_stop_line > 0 and self.control_state == CONTROL_STATE_STOPPING:
                #     # test code to give some info on the distance to the red light stop line waypoint
                #     rospy.loginfo("[test] Distance to red light stop line = " + str(distance_to_stop_line) + ",
                #                   current vel = " + str(self.current_velocity) +
                #                   ", target vel = " + str(start_point_velocity))

                """
                    Act on the current state of the car
                """
                if self.control_state == CONTROL_STATE_STOPPING:
                    # smoothly stop over the waypoints up to next_red_light waypoint
                    # setting desired velocity at each
                    for i in range(self.closest_waypoint, self.next_stop_line + 1):
                        # get the distance to the i-th way point
                        i_point_distance = self.distance(self.waypoints, self.closest_waypoint, i)
                        if (distance_to_stop_line - STOP_LINE_OFFSET) > 0:
                            i_point_target_velocity = i_point_distance
                            i_point_target_velocity /= (distance_to_stop_line - STOP_LINE_OFFSET)
                            i_point_target_velocity *= (start_point_velocity * -1)
                            i_point_target_velocity += start_point_velocity
                        else:
                            i_point_target_velocity = -10.0     # negative stops car 'creep' when stopped
                        self.set_waypoint_velocity(self.waypoints, i, i_point_target_velocity)
                else:
                    # just set the following waypoints to reference velocity
                    # speed controllers will sort out how to get to this desired velocity
                    for i in range(self.closest_waypoint, self.closest_waypoint + LOOKAHEAD_WPS):
                        if i < len(self.waypoints):
                            self.set_waypoint_velocity(self.waypoints, i, REFERENCE_VELOCITY)

            # now publish the waypoints - refactored from pose_cb
            # get waypoints ahead of the car
            # this currently sends as many as are available
            # should this fail if there aren't enough waypoints and just wait until there area enough?
            waypoints_ahead = []
            n_waypoints = len(self.waypoints)  # can only get this many waypoints
            if n_waypoints > LOOKAHEAD_WPS:
                n_waypoints = LOOKAHEAD_WPS  # max waypoints to pass over
            for i in range(n_waypoints):
                # check that the waypoints we want are in the range of the waypoint array
                if self.closest_waypoint + i < len(self.waypoints):
                    waypoints_ahead.append(self.waypoints[self.closest_waypoint + i])

            # structure the data to match the expected styx_msgs/Lane form
            lane = Lane()
            lane.waypoints = waypoints_ahead  # list of waypoints ahead of the car
            lane.header.stamp = rospy.Time(0)  # timestamp
            # publish the waypoints list
            self.final_waypoints_pub.publish(lane)

            rate.sleep()

    def pose_cb(self, msg):
        """
        Receive current car position
        Publish future waypoints

        :param msg:
        :return:
        """
        # Implement

        # msg will be a geometry_msgs/PoseStamped message
        # extract the current car x, y
        self.pose_x = msg.pose.position.x
        self.pose_y = msg.pose.position.y
        orientation = msg.pose.orientation
        euler = tf.transformations.euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w])
        self.theta = euler[2]

    def waypoints_cb(self, waypoints):
        """
        Callback function used to receive and store waypoints from base_waypoints topic
        :param waypoints:
        :return:
        """
        # Implement

        # receive waypoints in message type styx_msgs/Lane form

        # save these, to use when pose_cb is called later
        self.waypoints = waypoints.waypoints

        # we only need the message once, unsubscribe as soon as we handled the message
        self.base_waypoints_sub.unregister()

    def traffic_cb(self, msg):
        """
        Callback function used to receive next red light waypoint
        :param msg:
        :return:
        """
        # Callback for /traffic_waypoint message. Implement
        # get the waypoint ref of the **stop line** for the next red light (-1 if none)
        self.next_stop_line = msg.data

    def obstacle_cb(self, msg):
        """
        Callback to handle obstacles - likely for future Udacity projects
        :param msg:
        :return:
        """
        # Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        """
        Get velocity for a given waypoint object
        :param waypoint:
        :return:
        """
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        """
        Set velocity for a given waypoint in the list of waypoints
        :param waypoints:
        :param waypoint:
        :param velocity:
        :return:
        """
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        """
        Calculate distance between two waypoints
        :param waypoints:
        :param wp1:
        :param wp2:
        :return:
        """
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def dbw_enabled_cb(self, msg):
        """
        Callback function to store whether the car is in manual or autonomous/drive-by-wire mode
        :param msg:
        :return:
        """
        self.dbw_enabled = msg.data

    def current_velocity_cb(self, msg):
        """
        Callback function to store the car current velocity
        :param msg:
        :return:
        """
        # store the current velocity TwistStamped message
        self.current_velocity = msg.twist.linear.x

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
