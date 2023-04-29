#!/usr/bin/env python3
#=====================
import copy
from geometry_msgs.msg import TransformStamped
import numpy as np
import rospy
from sensor_msgs.msg import Imu
import tf2_ros
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

# Custom libraries
from body_pose_control.body_pose_valid import body_pose_valid
from body_pose_control.identity_pose import identity_pose
from system_globals import frames
from topic_messages import topics
from walking_gait.msg import GaitStates
from walking_gait.init_gait_states import init_gait_states

frame = frames.frames()
top = topics.topics()

# Get global parameters
NUM_LEGS = rospy.get_param( 'num_legs', default=6 )
L1 = rospy.get_param( 'L1', default=0.03226 )
L2 = rospy.get_param( 'L2', default=0.090 )
L3 = rospy.get_param( 'L3', default=0.113 )

#==============================================================================
class pose_gravity_compensation_node():
    def __init__(self):
        rospy.init_node("pose_gravity_compensation_node", anonymous=True)

        self.frame_input = rospy.get_param( f"{rospy.get_name()}/frame_in", default=frame.body_commanded )
        self.frame_output = rospy.get_param( f"{rospy.get_name()}/frame_out", default=frame.body_gravity_adjust )

        # Subscribe to our gait states to make sure we have up-to-date gait kinematics
        self.gait_states = init_gait_states( NUM_LEGS )
        rospy.Subscriber( top.gait_state, GaitStates, self.get_gait_states )

        # Subscribe to the filtered IMU data
        self.imu_filtered = Imu()
        rospy.Subscriber( top.imu_data_filtered, Imu, self.get_filtered_imu )

        # Control how fast we update the pose
        self.loop_hz = 30
        self.dt = 1 / self.loop_hz
        self.rate = rospy.Rate( self.loop_hz )

        self.Kp = rospy.get_param( f"{rospy.get_name()}/Kp", default=1.0 ) # Proportional gain value for this controller
        self.Ki = rospy.get_param( f"{rospy.get_name()}/Ki", default=0.0 ) # Integrator gain value for this controller
        self.Kd = rospy.get_param( f"{rospy.get_name()}/Kd", default=0.0 ) # Derivative gain value for this controller

        # Store off the integrand from the controller
        self.roll_integrate = 0.0
        self.pitch_integrate = 0.0

        # Store the previous state error so we can take the derivative
        self.roll_error_prev = 0.0
        self.pitch_error_prev = 0.0

        # Initilialize the commanded body pose
        self.pose_commanded = TransformStamped()
        self.pose_commanded.header.frame_id = frame.ground
        self.pose_commanded.child_frame_id = self.frame_input
        self.pose_commanded.transform = identity_pose()

        # Store our Euler angles so we don't have to extract them each loop
        self.roll_commanded = 0.0
        self.pitch_commanded = 0.0
        self.yaw_commanded = 0.0

        # Initilialize the gravity-compensated body pose
        self.pose_gravity = TransformStamped()
        self.pose_gravity.header.frame_id = frame.ground
        self.pose_gravity.child_frame_id = self.frame_output
        self.pose_gravity.transform = identity_pose()

        # Set up a tf listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener( self.tfBuffer )

        # Set up broadcaster and publisher
        self.tf2_broadcaster = tf2_ros.TransformBroadcaster()
        self.pub = rospy.Publisher( top.body_pose_gravity_adjusted, TransformStamped, queue_size=1 )

    #======================================================
    def get_gait_states( self, gait_states ):
        self.gait_states = copy.deepcopy( gait_states )

    #======================================================
    def get_filtered_imu( self, imu_filtered ):
        self.imu_filtered = copy.deepcopy( imu_filtered )

    #======================================================
    def gravity_compensation_pose( self ):

        while not rospy.is_shutdown():

            self.update_commanded_pose()

            # Calculate the desired pose orientation to compensate for gravity
            self.control_pose()

            # Send the compensated body pose out
            self.pose_gravity.header.stamp = rospy.Time.now()
            self.tf2_broadcaster.sendTransform( self.pose_gravity )
            self.pub.publish( self.pose_gravity )

            # Wait until it's time to loop around
            self.rate.sleep()

    #======================================================
    def update_commanded_pose( self ):
        # Get the latest commanded tf
        try:
            pose_commanded = self.tfBuffer.lookup_transform( frame.ground,
                                                             self.frame_input,
                                                             rospy.Time(0) )

            if pose_commanded.transform != self.pose_commanded.transform:
                self.pose_commanded = pose_commanded
                # Update the Euler angles
                (self.roll_commanded, self.pitch_commanded, self.yaw_commanded) = \
                    euler_from_quaternion( [ self.pose_commanded.transform.rotation.x,
                                             self.pose_commanded.transform.rotation.y,
                                             self.pose_commanded.transform.rotation.z,
                                             self.pose_commanded.transform.rotation.w ] )

        except:
            rospy.logwarn_throttle( 10, f"Could not get T_ground2body_commanded in {rospy.get_name()}. Using previous transformation")

    #======================================================
    def control_pose( self ):

        # Break measured orientation into Euler angles, so we can neglect yaw
        (roll_meas, pitch_meas, yaw_meas) = euler_from_quaternion(
                                                [ self.imu_filtered.orientation.x,
                                                  self.imu_filtered.orientation.y,
                                                  self.imu_filtered.orientation.z,
                                                  self.imu_filtered.orientation.w ] )

        # Calculate the state error
        roll_error = self.roll_commanded - roll_meas
        pitch_error = self.pitch_commanded - pitch_meas

        roll_error_dot = ( roll_error - self.roll_error_prev ) * self.loop_hz
        pitch_error_dot = ( pitch_error - self.pitch_error_prev ) * self.loop_hz

        roll_integrate = self.roll_integrate + roll_error *  self.dt
        pitch_integrate = self.pitch_integrate + pitch_error *  self.dt

        # Sum up the control components
        roll_grav  = roll_error  * self.Kp + roll_error_dot  * self.Kd + roll_integrate  * self.Ki
        pitch_grav = pitch_error * self.Kp + pitch_error_dot * self.Kd + pitch_integrate * self.Ki

        # Save off the error
        self.roll_error_prev = roll_error
        self.pitch_error_prev = pitch_error

        # We want to start with the commanded pose
        pose_gravity = TransformStamped()
        pose_gravity.transform = copy.deepcopy( self.pose_commanded.transform )

        # In case we are at the kinematic limit, try scaling the desired pose a bit until it fits,
        # or until we give up
        valid_pose = False
        scale = 1
        while not valid_pose and scale > 0.05:

            # Wrap Euler angles back into new body pose
            quat = quaternion_from_euler( self.roll_commanded + roll_grav,
                                          self.pitch_commanded + pitch_grav,
                                          self.yaw_commanded )
            pose_gravity.transform.rotation.x = quat[0]
            pose_gravity.transform.rotation.y = quat[1]
            pose_gravity.transform.rotation.z = quat[2]
            pose_gravity.transform.rotation.w = quat[3]

            # Make sure this pose is good before sending it out
            valid_pose = body_pose_valid( pose_gravity.transform,
                                          self.gait_states.foot_center,
                                          self.gait_states.min_stride_length,
                                          self.gait_states.max_stride_length,
                                          [L1,L2,L3] )
            if valid_pose:
                # This new pose is good, stash our states
                self.pose_gravity.transform = pose_gravity.transform
                self.roll_integrate = roll_integrate * scale
                self.pitch_integrate = pitch_integrate * scale
            else:
                # Try shrinking the angle compensation and trying again
                scale /= 2
                roll_grav *= scale
                pitch_grav *= scale

        return

#==============================================================================
if __name__ == "__main__":
    try:
        pg = pose_gravity_compensation_node()
        pg.gravity_compensation_pose()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
