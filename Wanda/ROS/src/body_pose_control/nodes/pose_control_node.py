#!/usr/bin/env python3
#=====================
import copy
import numpy as np
import rospy
from geometry_msgs.msg import TransformStamped
import tf2_ros

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
class pose_control_node():
    def __init__(self):
        rospy.init_node( "pose_control_node", anonymous=True )

        # Subscribe to our gait states to make sure we have up-to-date gait kinematics
        self.gait_states = init_gait_states( NUM_LEGS )
        rospy.Subscriber( top.gait_state, GaitStates, self.get_gait_states )

        self.frame_out = rospy.get_param( f"{rospy.get_name()}/frame_out", default=frame.body_commanded )

        self.pose = TransformStamped()
        self.pose.header.frame_id = frame.ground
        self.pose.child_frame_id = self.frame_out
        self.pose.transform = identity_pose()

        # Set up tf2 broadcaster/listener to push/pull the body pose
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener( self.tfBuffer )
        self.tf2_broadcaster = tf2_ros.TransformBroadcaster()

        self.loop_hz = 30
        self.period = 1 / self.loop_hz
        self.rate = rospy.Rate( self.loop_hz )

        self.pub = rospy.Publisher( top.body_pose_commanded, TransformStamped, queue_size=1 )

        self.stand_up_height = 0.05

        # If we are running with the simulation, wait for the clock to turn on
        while rospy.get_time() == 0 and not rospy.is_shutdown():
            rospy.loginfo_once( "pose_control_node.py waiting for /clock time to be published")

        self.stand_up()
        self.stay_standing()

    #======================================================
    def get_gait_states( self, gait_states ):
        self.gait_states = copy.deepcopy( gait_states )

    #======================================================
    def stand_up( self ):

        # Initialize the robot to start on the ground
        self.pose.header.stamp = rospy.Time.now()
        desired_speed = 0.01 # m/s

        while not rospy.is_shutdown() and self.pose.transform.translation.z < self.stand_up_height:

            # Grab current pose, in case another node updated it
            self.get_transform()

            # Move the robot vertically (the ground moves down relative to the body)
            self.pose.header.stamp = rospy.Time.now()
            self.pose.transform.translation.z += desired_speed * self.period # Push the ground down further
            self.pose.transform.translation.z = np.min( [self.stand_up_height, self.pose.transform.translation.z] ) # Make sure we don't go too far

            if body_pose_valid( self.pose.transform,
                                self.gait_states.foot_center,
                                self.gait_states.min_stride_length,
                                self.gait_states.max_stride_length,
                                [L1,L2,L3] ):

                self.tf2_broadcaster.sendTransform( self.pose )
                self.pub.publish( self.pose )

            # Wait until the next update time
            self.rate.sleep()

        # Send one last command to make sure we're at the actual desired height
        self.pose.header.stamp = rospy.Time.now()
        self.pose.transform.translation.z = self.stand_up_height
        self.tf2_broadcaster.sendTransform( self.pose )
        self.pub.publish( self.pose )

    #======================================================
    def stay_standing( self ):
        # Maintain old pose, don't move it
        while not rospy.is_shutdown():

            # Grab current pose, in case another node updated it
            self.get_transform()

            self.pose.header.stamp = rospy.Time.now()

            self.tf2_broadcaster.sendTransform( self.pose )
            self.pub.publish( self.pose )

            self.rate.sleep()

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
        pc = pose_control_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
