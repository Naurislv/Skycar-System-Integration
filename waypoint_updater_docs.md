## Waypoint Updating

Waypoint updating is done in `waypoint_update.py` and is responsible for the following key actions:

* Receiving initial waypoints from the `/base_waypoints` topic and storing this map for use in path calculations
* Receiving traffic light status information from the `/traffic_waypoint` topic
* Creating a path which will safely drive the car, including slowing or stopping the car in the run up to a red light
* Publishing the required waypoints to the `/final_waypoints` topic (after which the path will be processed by the `waypoint_follower` node)

### Initial waypoints
The receipt of initial waypoints is from the `/base_waypoints` topic and is handled by the `waypoints_cb` callback function.  This function simply stores the waypoints to a class variable and unsubscribes from `/base_waypoints` after receiving this data.  

The unsubscribe is used for performance reasons - the map of the track does not change over time and repeatedly receiving data from this topic will waste CPU time.

### Receiving traffic light status
Traffic light status information is received from the `/traffic_waypoint`topic.  The message is received by `traffic_cb` and stored in a class variable.

The traffic light status is simply an integer which indicates either the waypoint reference of the next traffic light, if that light is red, or -1 if the next light is green.

### Additional topics
The `waypoint_updater` node also subscribes to topics for the current position of the car (`/current_pose`), current velocity (`/current_velocity`) and whether the car is in autonomous mode (`/vehicle/dbw_enabled`).  These addition pieces of information are used in the path/velocity calculation process.

### Creating a path (velocities)
The path creation process is to set the target velocities for upcoming waypoints, to either smoothly bring the car to a stop for a red light, or keep the car driving at a reference speed.

This path planning is done with a finite state machine with just 2 states:

* _DRIVING_ state, where the car is aiming to reach or remain at the reference velocity
* _STOPPING_ state, where the car is aiming to reduce speed to a stop at the upcoming stop line

#### State choice/transitions
The choice of state is achieved by the following decisions:

* If the upcoming light is green, assign the _DRIVING_ state
* If the upcoming light is red and the distance to the light is greater than the distance required for a comfortable stop, assign the _DRIVING_ state 
* If the upcoming light is red and the distance to the stop line is less than the distance required to stop _at maximum braking_, assign the _DRIVING_ state
* If the upcoming light is red and neither of the previous conditions applies, assign the _STOPPING_ state

Clearly, if the upcoming light is green, the car should continue driving at a steady speed.  (Whilst it would be possible to slow down in anticipation of a green light turning red, it is assumed that the cycle times of traffic lights will be set to match the expected speed of traffic on the particular roadway.)  

If the upcoming light is red, but the light is beyond the distance required for a smooth and comfortable stop, the car will continue driving at a steady speed.  The calculation of the distance required for a comfortable stop is based on the current vehicle speed and a setting for a comfortable deceleration rate (`COMFORTABLE_DECEL`) and the following relationships:
* If the car is travelling at velocity __V__, it will travel distance __d__ = __Vt__ in time __t__
* If the comfortable deceleration rate is __C__, the car can decelerate to __V=0__ in time __t = V/C__
* Therefore the requirement is that __d/V < V/C__
* Rearranging gives __d < (V*V)/C__
If this condition is true, the car has is close enough to the red light that it should start to decelerate to a stop (4th condition above). Otherwise (if __d > (V*V)/C)__), the car will continue driving at a steady speed.

However, it may be the case that the light changes when the car is too close to the stop line to be able to stop at all, because there is a maximum deceleration possible (`MAXIMUM_DECEL`).  If the car is beyond the __d__ for the above calculation with maximum deceleration, the car should continue through the intersection to avoid stopping in an unsafe position.  
Note that some flexibility is given by the `MIN_EMERGENCY_VELOCITY` parameter, which allows the car to overshoot the stop line, if the speed is very low - this avoids the case of the car attempting to continue when it is already at a slow speed and cannot safely clear the intersection.

The 4th condition is the __d < (V*V)/C__ case, where the car is close enough to the red light to be slowing to a stop.

The state is set at ~10Hz, after which the velocity planning takes place.

#### Velocity planning - _DRIVING_ state
The velocity planning for the _DRIVING_ state is simply to set the upcoming waypoints to the reference velocity.  The accelerator PID controller will then take over the process of accelerating the car to the desired speed.

#### Velocity planning - _SLOWING_ state
The velocity planning for the _SLOWING_ state is to assign velocities to the waypoints up to stop line which decrease linearly from the current car velocity to zero.

#### Stop line location
The waypoint location of the red light is the location of the stop line on the entry to the intersection.  Using this exact location was found to lead to the car entering the intersection and therefore an offset parameter was used to adjust the point at which the car stops.  

The offset of the intersection stop line was determined empirically and is held in the `STOP_LINE_OFFSET` constant.  (Note that in practice this is will vary and additional map data/road sensing would be required to determine the stop location accurately.)

The `STOP_LINE_OFFSET` is incorporated into the distance calculation to determine when the car should start to slow.  It is also closely linked to the `MIN_STOP_DISTANCE` constant, which is used in the calculation of when the car can no longer stop safely.

### Publishing waypoints
The final calculated waypoints are published to the `/final_waypoints` topic, for processing by the `waypoints_follower` node.

