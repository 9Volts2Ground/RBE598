## Intro

The `manual_state_control_node` is designed to control the robot manually using an off-the-shelf videogame controller. This node can now control the walking gait twist, the body pose relative to the ground, and the seeker azimuth and elevation angles.

## Inputs

This node subscribes to the `joy` topic, of type `sensor_msgs.Joy`. This topic is generated from the ROS Noetic `joy` package. More information can be found here: http://wiki.ros.org/joy This node also calls some functions that use global system parameters.

## Outputs

This node publishes the following topics, at the same rate the subscribed `joy` topic is received:

- `walk_twist` topic of type `geometry_msgs.TwistWithCovarianceStamped`. This twist vector is the desired walking velocity vector, defined in the ground frame.
- `ground_body_transform` topic of type `geometry_msgs.TransformStamped`. This is the transformation between the ground frame and the robot body frame. In addition to publishing this topic, this transformation is broadcast as a `tf` frame.
- `seeker_states` topic of type `sensor_msgs.JointState`. This topic contains the desired azimuth and elevation angles for the seeker. In addition to publishing this topic, this transformation is broadcast as a `tf` frame.

## Launch

To run this node, the `manual_control.launch` file can be called using the following command:

`roslaunch manual_control manual_control.launch`

This launch file loads in the parameter files `hardware_constants.yaml` and `servos.yaml` from `system_globals/params`. These parameter files include hardware-based information used by this node, like static hardware lengths and servo angle limitations.

To generate a `joy` topic that this node subscribes to, the `joy.launch` file can be called by using the following command:

`roslaunch manual_control joy.launch`

Note: if multiple game controllers are found by the system, the `jsX` number should be updated appropriately in `joy.launch`. See the documentaion for the `joy` node linked above for more details.

## Control Modes

There are 11 DoF to control on the robot. These include:

- Walking gait twist linear X and Y and angular Z (3)
- Body pose linear position relative to the ground (X, Y, and Z) (3)
- Body pose orientation relative to the ground (roll, pitch, and yaw) (3)
- Seeker azimuth and elevation angles (2)

The two joysticks and the D-pad on an XBox controller gives only 6 DoF directly. Therefore, two "modes" are used for the two joysticks, giving 10 DoF. Using the triggers together as another DoF, this combined gives enough DoF to control the full robot. The joystick modes are controlled by toggling the right/left shoulder buttons. These modes are:

- Gait: controls elements of the walking gait commanded twist vector. This is the default.
- Pose: controls elements of the robot body pose relative to the ground.

The **left shoulder button** controls the mode for the left joystick, and similarly the **right shoulder button** controls the mode for the right joystick. Note that, because of the limitations of the XBox controller, this means that users cannot control the linear walking gait vector and the linear body pose at the same time. Similarly, users cannot control the body pose orientation and the walking gait angular twist at the same time.

## Left Joystick

### Gait Mode:

While in Gait mode, the left joystick controls the linear components of the walking gait commanded twist vector. Forward increments the X component of the linear velocity twist vector, and left increments the Y component of the linear velocity twist vector.

### Pose Mode:

While in Pose mode, the left joystick controls the planar linear components of the body pose. Forward increments the X component of the body pose, and left increments the Y component of the body pose.

## Right Joystick

### Gait Mode:

While in Gait mode, the right joystick controls both the angular component of the walking gait twist vector, as well as the seeker elevation angle. Forward decrements the seeker elevation angle (points down), and left increments the angular Z component of the walking gait twist (spin counter-clockwise).

### Pose Mode:

While in Pose mode, the right joystick controls the roll and pitch orientation components of the body pose relative to the ground. Forward increments the pitch angle of the body pose (nose down), and right increments the body pose roll angle (starboard down).

## D-Pad

The D-Pad is not subject to this multi-mode scheme. Forward on the D-Pad increments the body pose height off the ground. Left on the D-Pad increments the body pose yaw angle (counter-clockwise).

## Left/Right Triggers

The left and right triggers are used together to control the seeker azimuth angle. The left trigger increments the azimuth angle (points left), while the right trigger decrements the azimuth angle (points right).

## A-Button

For simplicity, the A button resets all elements of the body pose, except the height off the ground frame. Note: this does not incrementally bring the angles back to neautral. Instead, it immediately commands the body to snap return to 0 linear X and Y, and 0 Euler angles. If done on hardware, this may be hard on the servos and should be done with caution.

## B-Button

The B button toggles the ultrasonic range sensor state. This sensor is turned off at initialization, but pressing the B button turns the sensor on; pressing it again turns it back off.

## Testing

Controllers can be found under `/dev/input/jsX`, where `X` is a number indicating which controller number to use.

To test controller number `X` to ensure it is connected and working properly, call: `sudo jstest /dev/input/jsX`.

If the controller does not have permissions to test, call: `sudo chmod a+rw /dev/input/jsX`.
