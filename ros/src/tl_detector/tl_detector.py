#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
import time

STATE_COUNT_THRESHOLD = 3

USE_GROUND_TRUTH = True         # Use the ground truth traffic light data on /vehicle/traffic_lights
                                # This is to allow a build of the final waypoint controllers before
                                #   the traffic light classification has been developed

# Parameters used when gathering example images for TL classifier training
GATHER_IMAGES = False           # Set to true to gather sample images
GATHER_DISTANCE_MAX = 150.0     # Distance to light at which to start gathering sample images
GATHER_DISTANCE_MIN = 20.0      # Distance to light at which to stop gathering sample images
GATHER_RED = False              # Gather red light examples
GATHER_YELLOW = True            # Gather yellow light examples
GATHER_GREEN = True             # Gather green light examples
GATHER_RATE = 2                 # Will take the counter mod this value, to gather frames without slamming CPU
GATHER_DATA_PATH = '/home/neil/Downloads/sim_ground_truth'     # where to put the gathered images (edit for local)

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.best_waypoint = 0

        self.gather_count = 0           # used to skip frames when gathering ground truth data

        self.stop_line_positions = self.config['stop_line_positions']
        self.stop_line_wp = []

        rospy.spin()

    def distance(self, pt1, pt2):
        """Calculates dinstace from one point to another in 2D"""
        delta_x = pt1.x - pt2.x
        delta_y = pt1.y - pt2.y
        return math.sqrt(delta_x * delta_x + delta_y * delta_y)

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        # update the light positions - do this once here, so that we can use it or each TL detection
        self.stop_line_wp = []
        for lpts in self.stop_line_positions:
            l_pos = self.get_closest_waypoint_to_point(lpts[0], lpts[1], self.waypoints)
            self.stop_line_wp.append(l_pos)

    def get_closest_waypoint_to_point(self, pos_x, pos_y, waypoints):
        """Identifies the closest path waypoint to the given position
           https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
        Returns:
            int: index of the closest waypoint in self.waypoints
        """

        best_waypoint = None

        def distance2D(x1, y1, x2, y2):
            """Calculates dinstace from one point to another in 2D"""
            delta_x = x1 - x2
            delta_y = y1 - y2
            return math.sqrt(delta_x * delta_x + delta_y * delta_y)

        wps = waypoints.waypoints
        min_dist = distance2D(pos_x, pos_y,
                                 wps[0].pose.pose.position.x,
                                 wps[0].pose.pose.position.y)

        for i, point in enumerate(wps):
            dist = distance2D(pos_x, pos_y,
                                 point.pose.pose.position.x,
                                 point.pose.pose.position.y)
            if dist < min_dist:
                best_waypoint = i
                min_dist = dist

        return best_waypoint

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint
        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg

        if USE_GROUND_TRUTH:
            light_wp, state = self.process_ground_truth_lights()

            if GATHER_IMAGES:
                # Gather some ground truth data here
                if (self.pose) and light_wp > 0:
                    # Are we close enough?
                    ld = self.distance(self.pose.pose.position, self.waypoints.waypoints[light_wp].pose.pose.position)
                    if ld <= (GATHER_DISTANCE_MAX) and (ld >= GATHER_DISTANCE_MIN):
                        # get the image data
                        self.gather_count += 1
                        if (self.gather_count % GATHER_RATE) == 0:
                            fwrite = False
                            gather_dir = GATHER_DATA_PATH + '/'
                            if (state == TrafficLight.RED) and GATHER_RED:
                                gather_dir += 'RED/'
                                fwrite = True
                            elif (state == TrafficLight.YELLOW) and GATHER_YELLOW:
                                gather_dir += 'YELLOW/'
                                fwrite = True
                            elif (state == TrafficLight.GREEN) and GATHER_GREEN:
                                gather_dir += 'GREEN/'
                                fwrite = True
                            if fwrite:
                                # write out the file
                                cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
                                cv_image = cv2.resize(cv_image, (400, 300))
                                cv2.imwrite(gather_dir + '{}.png'.format(time.time()), cv_image)
                                rospy.loginfo("[test] Gathering ground truth for state %s (%s)", state, self.gather_count)


        else:
            light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """

        best_waypoint = self.best_waypoint
        if self.waypoints is not None:
            waypoints = self.waypoints.waypoints
            min_dist = self.distance(pose.position, waypoints[0].pose.pose.position)
            for i, point in enumerate(waypoints):
                dist = self.distance(pose.position, point.pose.pose.position)
                if dist < min_dist:
                    best_waypoint = i
                    min_dist = dist
            self.best_waypoint = best_waypoint

        return best_waypoint

    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use tranform and rotation to calculate 2D position of light in image

        x = 0
        y = 0

        return (x, y)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        self.camera_image.encoding = "rgb8"
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        x, y = self.project_to_image_plane(light.pose.pose.position)

        #TODO use light location to zoom in on traffic light in image

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)

            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        #light_positions = self.config['light_positions']
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)

        if light:
            state = self.get_light_state(light)
            return light_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

    def process_ground_truth_lights(self):
        """
        Finds the closest traffic light in the list of ground truth lights,
        then returns the state for that light.

        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        next_traffic_light_waypoint = -1
        next_traffic_light_state = TrafficLight.UNKNOWN

        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
            #rospy.loginfo("[test] self.pose.pose: %s", self.pose.pose)
            #rospy.loginfo("[test] car_position: %s", car_position)
            if car_position:
                # Have a car position, now need the nearest light
                #rospy.loginfo("[test] lights array length: " + str(len(self.lights)))
                #rospy.loginfo("[test] first light state : " + str(self.lights[0].state))
                #rospy.loginfo("[test] first light pose  : " + str(self.lights[0].pose.pose))
                #rospy.loginfo("[test] first light pose.x  : " + str(self.lights[0].pose.pose.position.x))
                #rospy.loginfo("[test] first light pose.y  : " + str(self.lights[0].pose.pose.position.y))
                #pass
                # loop over the lights

                for l in self.lights:
                    waypoint_nearest_light = self.get_closest_waypoint(l.pose.pose)
                    #rospy.loginfo("[test] light waypoint: %s", waypoint_nearest_light)
                    if waypoint_nearest_light > car_position:
                        next_traffic_light_waypoint = waypoint_nearest_light
                        next_traffic_light_state = l.state
                        #rospy.loginfo("[test] next light waypoint is %s, in state = " + str(l.state),
                        #              next_traffic_light_waypoint)
                        break

        best_stop_line = -1
        if next_traffic_light_waypoint > 0:
            # find the stop line location nearest to this traffic light
            stop_distance = 10000           # here distance is just a count of waypoints, not meters
            for stop_line in self.stop_line_wp:
                if stop_line < next_traffic_light_waypoint:
                    # stop line is before the traffic light
                    if (next_traffic_light_waypoint - stop_line) < stop_distance:
                        stop_distance = (next_traffic_light_waypoint - stop_line)
                        best_stop_line = stop_line

        if best_stop_line > 0:
            return best_stop_line, next_traffic_light_state
        else:
            return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
