## Drive-By-Wire
Controlling the speed and yaw of the car is done by the drive-by-wire (dbw) node implemented in `dbw_node.py`. The dwb node makes use of several
controllers and a low-pass filter to control the cars throttle, brake, and steering such that the car follows the desired path
on the road and stops at red lights if desired. The controllers are reset if drive by wire is disabled through the `dbw_enabled` topic.
Asided from the `dbw_enabled` topic, the dbw_node subscribes to the `/current_velocity` and the `/twist_cmd` topic
to get the current as well as the desired velocity, respectively.


### Speed control
The actual control algorithm is implemented in `twist-controller.py`. For accurate speed control, a combination of two PID controllers
and a low-pass filter is used. First, the raw actual acceleration is computed from the difference between desired and actual angular velocity.
This raw actual acceleration is filtered using a low-pass filter to get a smoother acceleration signal.   
Next, the first PID controller is used to compute the desired acceleration to based on the velocity error. Then an acceleration error is computed,
which is then fed into the second PID controller to compute a throttle setting.  The advantage of this two stage approach is that we can
nicely limit the throttle value to a range between 0 and 1.

### Yaw control
To compute an appropriate steering command, the yaw-controller provided by Udacity is used.

### Reseting the controller
The drive by wire node is completely deactivated if the message of the `dbw_enabled` topic is false. In addition, The PID controllers are reset to prevent
accumulation of errors in the I-part of the controller.

### Publishing throttle, brake, and steering
After the control algorithms have computed the optimal actuations for throttle, brake, and steering, the respective values are published to
`/vehicle/throttle_cmd`, `/vehicle/brake_cmd`, and  `/vehicle/steering_cmd` topic by the `dbw_node`.
