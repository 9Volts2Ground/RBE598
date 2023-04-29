#!/usr/bin/env python3
#=====================
import copy
import rospy
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf.transformations import quaternion_slerp

# Custom libraries
from body_pose_control.body_pose_valid import body_pose_valid
from body_pose_control.identity_pose import identity_pose
from system_globals import frames
from topic_messages import topics
from walking_gait.msg import GaitStates
from walking_gait.init_gait_states import init_gait_states

# Get global parameters
NUM_LEGS = rospy.get_param( 'num_legs', default=6 )
L1 = rospy.get_param( 'L1', default=0.03226 )
L2 = rospy.get_param( 'L2', default=0.090 )
L3 = rospy.get_param( 'L3', default=0.113 )

frame = frames.frames()
top = topics.topics()

#==============================================================================
class pose_filter_node():
    def __init__(self):
        rospy.init_node( "pose_filter_node", anonymous=True )

        self.frame_input = rospy.get_param( f"{rospy.get_name()}/frame_in", default=frame.body_gravity_adjust )
        self.frame_output = rospy.get_param( f"{rospy.get_name()}/frame_out", default=frame.body )

        # Initialize the body pose
        self.pose = TransformStamped()
        self.pose.header.frame_id = frame.ground
        self.pose.child_frame_id = self.frame_output
        self.pose.transform = identity_pose()

        self.pose_in = TransformStamped()
        self.pose_in.header.frame_id = frame.ground
        self.pose_in.child_frame_id = self.frame_input
        self.pose_in.transform = identity_pose()

        # Filter parameters
        self.filter_gain = 0.2

        # Subscribe to our gait states to make sure we have up-to-date gait kinematics
        self.gait_states = init_gait_states( NUM_LEGS )
        rospy.Subscriber( top.gait_state, GaitStates, self.get_gait_states )

        # Listen to the body pose we want to filter
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener( self.tfBuffer )

        # Set up the publisher and broadcaster
        self.tf2_broadcaster = tf2_ros.TransformBroadcaster()
        self.pub = rospy.Publisher( top.body_pose_filtered, TransformStamped, queue_size=1 )
        self.filter_pose()

    #======================================================
    def get_gait_states( self, gait_states ):
        self.gait_states = copy.deepcopy( gait_states )

    #======================================================
    def filter_pose( self ):

        rate = rospy.Rate( 30 )

        while not rospy.is_shutdown():

            try:
                tran = self.tfBuffer.lookup_transform( frame.ground,
                                                       self.frame_input,
                                                       rospy.Time() )
                self.pose_in = tran
            except:
                rospy.logwarn_throttle( 10, f"Could not get transform in {rospy.get_name()}. Using previous transformation" )

            # Only bother to update it if the new pose is different
            if self.pose_in.transform != self.pose.transform:

                # Filter our pose, temp variable
                pose_filtered = copy.deepcopy( self.pose )
                pose_in = copy.deepcopy( self.pose_in )

                # Filter the orientation using SLERP
                q_old = [pose_filtered.transform.rotation.x,
                         pose_filtered.transform.rotation.y,
                         pose_filtered.transform.rotation.z,
                         pose_filtered.transform.rotation.w]

                q_new = [pose_in.transform.rotation.x,
                         pose_in.transform.rotation.y,
                         pose_in.transform.rotation.z,
                         pose_in.transform.rotation.w]

                q_filtered = quaternion_slerp( q_old, q_new, self.filter_gain )

                pose_filtered.transform.rotation.x = q_filtered[0]
                pose_filtered.transform.rotation.y = q_filtered[1]
                pose_filtered.transform.rotation.z = q_filtered[2]
                pose_filtered.transform.rotation.w = q_filtered[3]

                # Filter the position
                pose_filtered.transform.translation.x = self.low_pass_filter(
                                                    pose_filtered.transform.translation.x,
                                                    pose_in.transform.translation.x,
                                                    self.filter_gain
                                                    )

                pose_filtered.transform.translation.y = self.low_pass_filter(
                                                    pose_filtered.transform.translation.y,
                                                    pose_in.transform.translation.y,
                                                    self.filter_gain
                                                    )

                pose_filtered.transform.translation.z = self.low_pass_filter(
                                                    pose_filtered.transform.translation.z,
                                                    pose_in.transform.translation.z,
                                                    self.filter_gain
                                                    )

                # Make sure to only keep it if the new pose is valid
                if body_pose_valid( pose_filtered.transform,
                                    self.gait_states.foot_center,
                                    self.gait_states.min_stride_length,
                                    self.gait_states.max_stride_length,
                                    [L1,L2,L3] ):

                    self.pose.transform = copy.deepcopy( pose_filtered.transform )

            # Send out the updated pose
            self.pose.header.stamp = rospy.Time.now()
            self.tf2_broadcaster.sendTransform( self.pose )
            self.pub.publish( self.pose )

            rate.sleep()

    #======================================================
    def low_pass_filter( self, state_current, state_old, period=None ):
        if period == None:
            # Use the default filter frequency if none is specified
            period = self.period

        state_new = period * state_old + (1-period) * state_current

        return state_new

#==============================================================================
if __name__ == "__main__":
    try:
        pf = pose_filter_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
