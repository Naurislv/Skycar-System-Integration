
import rospy                                    # will need access to parameters and system time
from pid import PID                             # PID controller provided
from yaw_controller import YawController        # Yaw controller provided


# original Udacity constants
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

# Tuning parameters for throttle/brake PID controller
PID_THROTTLE_BRAKE_P = 0.9
PID_THROTTLE_BRAKE_I = 0.0005
PID_THROTTLE_BRAKE_D = 0.07

class Controller(object):
    def __init__(self, *args, **kwargs):
        rospy.loginfo('TwistController: Start init')
        self.sampling_rate = kwargs["sampling_rate"]
        self.decel_limit = kwargs["decel_limit"]
        self.accel_limit = kwargs["accel_limit"]
        # brake_deadband is the interval in which the brake would be ignored
        # the car would just be allowed to slow by itself/coast to a slower speed
        self.brake_deadband = kwargs["brake_deadband"]
        self.vehicle_mass = kwargs["vehicle_mass"]
        self.fuel_capacity = kwargs["fuel_capacity"]
        self.wheel_radius = kwargs["wheel_radius"]
        # bunch of parameters to use for the Yaw (steering) controller
        self.wheel_base = kwargs["wheel_base"]
        self.steer_ratio = kwargs["steer_ratio"]
        self.max_lat_accel = kwargs["max_lat_accel"]
        self.max_steer_angle = kwargs["max_steer_angle"]


        self.delta_t = 1/self.sampling_rate
        self.brake_torque_const = (self.vehicle_mass + self.fuel_capacity \
            * GAS_DENSITY) * self.wheel_radius


        # Initialise speed PID, with tuning parameters
        # Will use this PID for the speed control
        self.pid_throttle_brake = PID(PID_THROTTLE_BRAKE_P,
                                      PID_THROTTLE_BRAKE_I,
                                      PID_THROTTLE_BRAKE_D,
                                      self.decel_limit,
                                      self.accel_limit)

        # Initialise Yaw controller - this gives steering values using
        # vehicle attributes/bicycle model
        # Need to have some minimum speed before steering is applied
        self.yaw_controller = YawController(wheel_base=self.wheel_base,
                                            steer_ratio=self.steer_ratio,
                                            min_speed=5.0,
                                            max_lat_accel=self.max_lat_accel,
                                            max_steer_angle=self.max_steer_angle)

        rospy.loginfo('TwistController: Complete init')
        rospy.loginfo('TwistController: Steer ratio = ' + str(self.steer_ratio))

    def control(self, required_vel_linear,
                required_vel_angular,
                current_vel_linear,
                current_vel_angular,
                **kwargs):

        throttle, brake, steering = 0.0, 0.0, 0.0

        # uncomment for debugging pruposes otherwise we write logs at 50 Hz
        #rospy.loginfo('TwistController: Control call at ' + str(rospy.get_time()))
        #rospy.loginfo('TwistController: ' + 'Desired linear/angular = ' + str(required_vel_linear) + ","
        #              + str(required_vel_angular) + ' Current = ' + str(current_vel_linear) + ',' + str(current_vel_angular))

        # calculate the difference (error) between desired and current linear velocity
        velocity_error = required_vel_linear - current_vel_linear
        pid_value = self.pid_throttle_brake.step(velocity_error, self.delta_t)
        if pid_value > 0:
            throttle = pid_value
            brake = 0.0
        else:
            throttle = 0.0
            if abs(pid_value) > self.brake_deadband:
                # don't bother braking unless over the deadband level
                brake = abs(pid_value) * self.brake_torque_const

        # steering - yaw controller takes desired linear, desired angular, current linear as params
        steering = self.yaw_controller.get_steering(required_vel_linear,
                                                    required_vel_angular,
                                                    current_vel_linear)

        # check that steering inputs aren't degrees - they're really not!
        # steering = math.degrees(steering)

        # uncomment for debugging
        #if throttle <> 0.0:
        #   rospy.loginfo('TwistController: Accelerating = ' + str(throttle))
        #if brake <> 0.0:
        #    rospy.loginfo('TwistController: Braking = ' + str(brake))
        #if abs(steering) <> 0.0:
        #    rospy.loginfo('TwistController: Steering = ' + str(steering))

        return throttle, brake, steering

    def reset(self):
        self.pid_throttle_brake.reset()
