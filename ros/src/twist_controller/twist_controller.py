
import rospy                                    # will need access to parameters and system time
from pid import PID                             # PID controller provided
from yaw_controller import YawController        # Yaw controller provided
import math

# original Udacity constants
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

# Tuning parameters for throttle/brake PID controller
PID_THROTTLE_BRAKE_P = 0.9
PID_THROTTLE_BRAKE_I = 0.0005
PID_THROTTLE_BRAKE_D = 0.07 # 0.01?

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        #pass
        rospy.loginfo('TwistController: Start init')

        # Initialise speed PID, with tuning parameters
        # Will use this PID for the speed control
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        self.pid_throttle_brake = PID(PID_THROTTLE_BRAKE_P, PID_THROTTLE_BRAKE_I, PID_THROTTLE_BRAKE_D,
                                      decel_limit, accel_limit)

        # Need a timestamp for the last time the Twist controller was called
        # Will use this for calculating delta-t (sample time) to pass to the PID controller
        self.last_call_time = None

        # brake_deadband is the interval in which the brake would be ignored
        # the car would just be allowed to slow by itself/coast to a slower speed
        self.brake_deadband = rospy.get_param('~brake_deadband', .1)

        # bunch of parameters to use for the Yaw (steering) controller - from dbw_node.py given code
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)


        # Initialise Yaw controller - this gives steering values using vehicle attributes/bicycle model
        self.yaw_controller  = YawController(wheel_base=wheel_base,
                                          steer_ratio=steer_ratio,
                                          min_speed=0.0,
                                          max_lat_accel=max_lat_accel,
                                          max_steer_angle=max_steer_angle)

        rospy.loginfo('TwistController: Complete init')

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        throttle, brake, steering = 0.0, 0.0, 0.0

        rospy.loginfo('TwistController: Control call at ' + str(rospy.get_time()))

        dbw_enabled = kwargs['dbw_enabled']             # is DBW enabled (vs manual driving)

        rospy.loginfo('TwistController: dbw_enabled = ' + str(dbw_enabled))

        twist_linear = kwargs['twist_cmd'].twist.linear.x     # values requested by the twist command/follower
        twist_angular = kwargs['twist_cmd'].twist.angular.z

        current_linear = kwargs['current_velocity'].twist.linear.x    # current values
        current_angular = kwargs['current_velocity'].twist.angular.z

        rospy.loginfo('TwistController: ' + 'Desired linear/angular = ' + str(twist_linear) + ","
                      + str(twist_angular) + ' Current = ' + str(current_linear) + ',' + str(current_angular))

        # task is to figure out which throttle/brake/steer to apply to move car towards requested linear/angular

        if dbw_enabled is False:
            # manual driving, so reset the PIDs (zero the integral count) for next time
            self.pid_throttle_brake.reset()

        if self.last_call_time is not None:
            # have a time stamp for the last call, so can calculate sample time since last input
            time_now = rospy.get_time()
            sample_time = time_now - self.last_call_time
            self.last_call_time = time_now

            rospy.loginfo('TwistController: Current time = ' + str(sample_time))

            # calculate the difference (error) between desired and current linear velocity
            velocity_error = twist_linear - current_linear
            pid_value = self.pid_throttle_brake.step(velocity_error, sample_time)
            if pid_value > 0:
                throttle = pid_value
                brake = 0.0
            else:
                throttle = 0.0
                if abs(pid_value) > self.brake_deadband:
                    # don't bother braking unless over the deadband level
                    brake = 0.0 - pid_value

            # steering - yaw controller takes desired linear, desired angular, current linear as params
            steering = self.yaw_controller.get_steering(twist_linear, twist_angular, current_linear)

        else:
            # no last call time
            self.last_call_time = rospy.get_time()       # use ROS to get system time
            throttle, brake, steering = 0.0, 0.0, 0.0                    # no inputs to the car systems yet...

        rospy.loginfo('TwistController: T/B/S output = ' + str(throttle) + ','
                      + str(brake) + ',' + str(steering))

        return throttle, brake, steering

        #return 0.5, brake, 0.5
