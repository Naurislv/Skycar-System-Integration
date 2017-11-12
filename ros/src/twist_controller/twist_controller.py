
import rospy                                    # will need access to parameters and system time
from pid import PID                             # PID controller provided
from yaw_controller import YawController        # Yaw controller provided
from lowpass import LowPassFilter

# original Udacity constants
GAS_DENSITY = 2.858
ONE_MPH = 0.44704

# Tuning parameters for throttle/brake PID controller
PID_VEL_P = 0.9
PID_VEL_I = 0.0005
PID_VEL_D = 0.07

#
PID_ACC_P = 0.4
PID_ACC_I = 0.05
PID_ACC_D = 0.0

LPF_ACCEL_TAU = 0.2

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

        self.past_vel_linear = 0.0
        self.current_accel = 0.0
        self.low_pass_filter_accel = LowPassFilter(LPF_ACCEL_TAU, self.delta_t)



        # Initialise speed PID, with tuning parameters
        # Will use this PID for the speed control
        self.pid_vel_linear = PID(PID_VEL_P, PID_VEL_I, PID_VEL_D,
                                  self.decel_limit, self.accel_limit)

        # second controller to get throttle signal between 0 and 1
        self.accel_pid = PID(PID_ACC_P, PID_ACC_I, PID_ACC_D, 0.0, 0.75)


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

    def control(self, required_vel_linear,required_vel_angular,
                current_vel_linear):

        throttle, brake, steering = 0.0, 0.0, 0.0

        # uncomment for debugging pruposes otherwise we write logs at 50 Hz
        #rospy.loginfo('TwistController: Control call at ' + str(rospy.get_time()))
        #rospy.loginfo('TwistController: ' + 'Desired linear/angular = ' + str(required_vel_linear) + ","
        #              + str(required_vel_angular) + ' Current = ' + str(current_vel_linear) + ',' + str(current_vel_angular))

        # calculate the difference (error) between desired and current linear velocity
        velocity_error = required_vel_linear - current_vel_linear

        # calculate current acceleration and smooth using lpf
        accel_temp = self.sampling_rate * (self.past_vel_linear - current_vel_linear)
        # update
        self.past_vel_linear = current_vel_linear
        self.low_pass_filter_accel.filt(accel_temp)
        self.current_accel = self.low_pass_filter_accel.get()

        # use velocity controller compute desired accelaration
        desired_accel = self.pid_vel_linear.step(velocity_error, self.delta_t)

        # TODO think about emergency brake command
        if desired_accel > 0.0:
            if desired_accel < self.accel_limit:
                throttle = self.accel_pid.step(desired_accel - self.current_accel, self.delta_t)
            else:
                throttle = self.accel_pid.step(self.accel_limit - self.current_accel, self.delta_t)
            brake = 0.0
        else:
            throttle = 0.0
            # reset just to be sure
            self.accel_pid.reset()
            if abs(desired_accel) > self.brake_deadband:
                # don't bother braking unless over the deadband level
                # make sure we do not brake to hard
                if abs(desired_accel) > abs(self.decel_limit):
                    brake = abs(self.decel_limit) * self.brake_torque_const
                else:
                    brake = abs(desired_accel) * self.brake_torque_const


        # steering - yaw controller takes desired linear, desired angular, current linear as params
        #steering = required_vel_angular * self.steer_ratio
        steering = self.yaw_controller.get_steering(required_vel_linear,
                                                    required_vel_angular,
                                                    current_vel_linear)

        # uncomment for debugging
        #if throttle <> 0.0:
        #   rospy.loginfo('TwistController: Accelerating = ' + str(throttle))
        #if brake <> 0.0:
        #    rospy.loginfo('TwistController: Braking = ' + str(brake))
        if abs(steering) <> 0.0:
            #rospy.loginfo('TwistController: Steering = ' + str(steering))
            rospy.loginfo('Veer: Steering = ' + str(steering) + ', required = ' + str(required_vel_angular))

        return throttle, brake, steering

    def reset(self):
        self.pid_vel_linear.reset()
