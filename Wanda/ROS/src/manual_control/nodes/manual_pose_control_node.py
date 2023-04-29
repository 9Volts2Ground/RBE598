#!/usr/bin/env python3
#=====================
import copy
import numpy as np
import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

# Standard ROS libraries
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Joy

# Custom libraries
from body_pose_control import body_pose_valid
from manual_control.game_controller import game_controller
from manual_control.stick_mode import stick_mode
from manual_control.update_joystick_mode import update_joystick_mode
from system_globals import frames
from system_globals import transformation_frames
from topic_messages import topics
from walking_gait.msg import GaitStates
from walking_gait.init_gait_states import init_gait_states

frame = frames.frames()
top = topics.topics()
trans = transformation_frames.transformation_frames()

NUM_LEGS = rospy.get_param( 'num_legs', default=6 )
L1 = rospy.get_param( 'L1', default=0.03226 )
L2 = rospy.get_param( 'L2', default=0.090 )
L3 = rospy.get_param( 'L3', default=0.113 )

DEG2RAD = np.pi / 180

#==============================================================================
class manual_pose_control_node():
    def __init__(self):
        rospy.init_node( "manual_pose_control_node", anonymous=True)

        # Initialize controller instance for parsing
        self.controller = game_controller()
        self.controller_previous = copy.deepcopy( self.controller )

        # Subscribe to our gait states to make sure we have up-to-date gait kinematics
        self.gait_states = init_gait_states( NUM_LEGS )
        rospy.Subscriber( top.gait_state, GaitStates, self.get_gait_states )

        self.frame_out = rospy.get_param( f"{rospy.get_name()}/frame_out", default=frame.body_commanded )

        # Initialize body pose state
        self.pose = TransformStamped()
        self.pose.header.frame_id = frame.ground
        self.pose.child_frame_id = self.frame_out
        self.clear_pose_linear()
        self.clear_pose_angular()

        # Set up tf2 broadcaster/listener to push/pull the body pose
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener( self.tfBuffer )
        self.tf2_broadcaster = tf2_ros.TransformBroadcaster()

        # Use the bumpers to switch each joystick control mode: walk vs pose
        self.l_stick_mode = stick_mode.WALK
        self.r_stick_mode = stick_mode.WALK

        self.pose_linear_rate = 0.02 # m/s
        self.pose_euler_rate = 15 * DEG2RAD # rad/s

        # Define publisher
        self.pub_pose = rospy.Publisher( top.body_pose_commanded, TransformStamped, queue_size = 1 )

        # If we are running with the simulation, wait for the clock to turn on
        while rospy.get_time() == 0 and not rospy.is_shutdown():
            rospy.loginfo_once( "manual_pose_control_node.py waiting for /clock time to be published")

        # Time stamp, used to integrate positions
        self.time_stamp = rospy.Time.now()
        self.t = rospy.get_time()
        self.t_previous = self.t
        self.dt = 0.0
        self.update_dt()

        # Initialize subscription to joy controller node
        rospy.Subscriber( "joy", Joy, self.manual_control )

    #======================================================
    def manual_control( self, joy ):
        '''
        This is the primary control flow, where we poll the controller inputs
        and set the commanded output signals'''

        # Parse Joy inputs
        self.controller_previous = copy.deepcopy( self.controller )
        self.controller.map_joy_to_controller( joy )

        # Calculate dt since last change
        self.update_dt()

        # Check if we are controling walking gait or body pose
        self.r_stick_mode, self.l_stick_mode = update_joystick_mode( self.controller,
                                                                     self.controller_previous,
                                                                     self.r_stick_mode,
                                                                     self.l_stick_mode )

        # Calculate commanded body pose
        self.calculate_body_pose()

    #======================================================
    def update_dt( self ):
        '''Calculate dt since last change'''
        self.time_stamp = rospy.Time.now()
        self.t = rospy.get_time()
        self.dt = self.t - self.t_previous
        self.t_previous = self.t

    #======================================================
    def calculate_body_pose( self ):
        ''' Convert /joy topic into commanded body pose'''

        # Grab the current status of this pose
        self.get_transform()

        self.pose.header.stamp = self.time_stamp
        pose_old = copy.deepcopy( self.pose.transform )

        #------------------------------------------
        # Allows the A button to reset the pose (but maintain walking height)
        if self.controller.a == 1 and self.controller_previous.a == 0:
            self.pose.transform.translation.x = 0.0
            self.pose.transform.translation.y = 0.0
            self.clear_pose_angular()

        else:
            # Let the user move the pose in some other way
            # -----------------------------------------
            # Handle angular pose
            # Grab the current Euler angles so we can adjust them
            (roll, pitch, yaw) = euler_from_quaternion( [ self.pose.transform.rotation.x,
                                                          self.pose.transform.rotation.y,
                                                          self.pose.transform.rotation.z,
                                                          self.pose.transform.rotation.w ] )
            yaw = self.set_body_yaw( yaw )

            if self.r_stick_mode == stick_mode.POSE:
                roll, pitch = self.set_roll_pitch( roll, pitch )

            # Wrap Euler angles back into new body pose
            quat = quaternion_from_euler( roll, pitch, yaw )
            self.pose.transform.rotation.x = quat[0]
            self.pose.transform.rotation.y = quat[1]
            self.pose.transform.rotation.z = quat[2]
            self.pose.transform.rotation.w = quat[3]

            # -----------------------------------------
            # Handle linear pose
            self.set_body_height()
            if self.l_stick_mode == stick_mode.POSE:
                self.update_linear_pose()

        if not body_pose_valid.body_pose_valid( self.pose.transform,
                                                self.gait_states.foot_center,
                                                self.gait_states.min_stride_length,
                                                self.gait_states.max_stride_length,
                                                [L1,L2,L3] ):
            # This isn't a good pose. Keep the old one
            self.pose.transform = pose_old

        # -----------------------------------------
        # Send out our calculated pose values
        self.tf2_broadcaster.sendTransform( self.pose )
        self.pub_pose.publish( self.pose )

    #======================================================
    def set_body_yaw( self, yaw ):
        # Positive yaw is counter-clockwise around the body
        yaw -= self.controller.d_pad.right_left * self.dt * self.pose_euler_rate
        return yaw

    #======================================================
    def set_roll_pitch( self, roll, pitch ):
        ''' Let the right stick adjust the orientation of the body'''

        roll += self.controller.r_stick.right_left * self.dt * self.pose_euler_rate
        pitch += self.controller.r_stick.fwd_back * self.dt * self.pose_euler_rate

        return roll, pitch

    #======================================================
    def set_body_height( self ):
        # D-pad controls body height. Up pushes the groud down from the body
        self.pose.transform.translation.z += self.controller.d_pad.fwd_back * self.dt * self.pose_linear_rate

    #======================================================
    def update_linear_pose( self ):
        self.pose.transform.translation.x += self.controller.l_stick.fwd_back * self.dt * self.pose_linear_rate

        # Positive body Y is to the left, joystick positive right
        self.pose.transform.translation.y -= self.controller.l_stick.right_left * self.dt * self.pose_linear_rate

    #======================================================
    def clear_pose_linear( self ):
        self.pose.transform.translation.x = 0.0
        self.pose.transform.translation.y = 0.0
        self.pose.transform.translation.z = 0.0

    #======================================================
    def clear_pose_angular( self ):
        self.pose.transform.rotation.x = 0.0
        self.pose.transform.rotation.y = 0.0
        self.pose.transform.rotation.z = 0.0
        self.pose.transform.rotation.w = 1.0

    #======================================================
    def get_gait_states( self, gait_states ):
        self.gait_states = copy.deepcopy( gait_states )

    #======================================================
    def get_transform( self ):
        try:
            tran = self.tfBuffer.lookup_transform( frame.ground,
                                                   self.frame_out,
                                                   rospy.Time() )
            self.pose = tran
        except:
            rospy.logwarn_throttle( 10, f"Could not get transform {self.frame_out} in {rospy.get_name()}. Using previous transformation" )

#==============================================================================
if __name__ == "__main__":
    try:
        pc = manual_pose_control_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
