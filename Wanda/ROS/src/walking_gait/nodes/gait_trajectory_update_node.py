#!/usr/bin/env python3
#=====================
import copy
import numpy as np
import rospy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistStamped
import tf2_ros

# Custom libraries and class instances
from body_pose_control.identity_pose import identity_pose
from system_globals import frames
from topic_messages import topics
from walking_gait.msg import FootStates
from walking_gait.msg import GaitStates
from walking_gait import gait
from walking_gait.init_gait_states import init_gait_states
from walking_gait.leg_states import leg_states
from walking_gait import rotation

#============================================================
# Grab global hardware parameters
#============================================================
NUM_LEGS = rospy.get_param( 'num_legs', default=6 )

# Initialize classes
frame = frames.frames()
top = topics.topics()

#==============================================================================
class gait_trajectory_update_node():
    def __init__(self):

        # Initialize ROS communication
        rospy.init_node( "gait_trajectory_update_node", anonymous=True )

        # Initialize our GaitState topic info
        self.gait_state = init_gait_states( NUM_LEGS )

        # Initialize our gait state class
        self.gt = gait.gait( NUM_LEGS, GaitStates.WAVE )
        self.phase = 0

        # Publish a state of each leg to its own topic
        self.pub = []
        self.leg_states = []
        for leg in range( NUM_LEGS ):
            self.pub.append( rospy.Publisher( top.leg_foot_states[leg], FootStates, queue_size = 1 ) )

            self.leg_states.append( leg_states( leg,
                                                self.gait_state.foot_center[leg],
                                                frame.foot_ground[leg] ) )

            self.leg_states[leg].set_foot_phase( self.gt.phase_offset[leg], self.gt.beta )

        # Sends out a transform for the foot in the ground frame
        self.tf2_broadcaster = tf2_ros.TransformBroadcaster()
        # Listens to the body pose to set the foot height
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener( self.tfBuffer )
        self.pose = TransformStamped()
        self.pose.header.frame_id = frame.ground
        self.pose.child_frame_id = frame.body
        self.pose.transform = identity_pose()

        # If we are running with the simulation, wait for the clock to turn on
        while rospy.get_time() == 0 and not rospy.is_shutdown():
            rospy.loginfo_once( "gait_trajectory_update_node.py waiting for /clock time to be published")

        # Initialize clock variables
        update_hz = 30
        self.rate = rospy.Rate( update_hz )
        self.dt = 1 / update_hz

        # Subscribe to the gait state control topic
        rospy.Subscriber( top.gait_state, GaitStates, self.get_gait_state )

        self.topic = rospy.get_param( f"{rospy.get_name()}/topic", default=top.walking_twist_filtered )

        # Subscribe to walk control after initializing all your stuff. Main input to this alg
        self.walk_twist = TwistStamped()
        rospy.Subscriber( self.topic, TwistStamped, self.get_gait_twist )

    #==============================================================================
    def get_gait_twist( self, walk_twist ):
        """Grab walk_twist topic data when published, store it as a local class member
        Args:
            walk_twist (TwistStamped): _description_
        """
        self.walk_twist = copy.deepcopy( walk_twist )

    #==============================================================================
    def gait_trajectory_update( self ):

        while not rospy.is_shutdown():
            self.update_phase()

            # Grab the body pose
            try:
                self.pose = self.tfBuffer.lookup_transform( frame.ground,
                                                                      frame.body,
                                                                      rospy.Time(0) )

                self.gt.update_foot_height( self.pose )
            except:
                rospy.logwarn_throttle( 10, f"Could not get T_ground2body in {rospy.get_name()}. Using previous transformation")

            # Calculate our new desired foot positions
            walk_twist_local = copy.deepcopy( self.walk_twist ) # Save off local copy for safety
            time_stamp = rospy.Time.now()
            for leg in range( NUM_LEGS ):
                self.gait_update( leg, walk_twist_local.twist )

                # Package leg state info into message
                self.leg_states[leg].foot_states.header.stamp = time_stamp
                self.pub[leg].publish( self.leg_states[leg].foot_states )

                # Publish foot position transformation, relative to ground frame
                self.leg_states[leg].foot_trans.header.stamp = time_stamp
                self.tf2_broadcaster.sendTransform( self.leg_states[leg].foot_trans )

            self.rate.sleep()

    #==============================================================================
    def update_phase( self ):
        # Find what part of the gait phase we are in
        max_foot_traveled = 0.0
        for leg in range( NUM_LEGS ):

            if not self.leg_states[leg].foot_off_ground:
                if self.leg_states[leg].foot_center_dist_delta > max_foot_traveled:
                    max_foot_traveled = self.leg_states[leg].foot_center_dist_delta

        # Increment the stride phase
        self.gt.increment_phase( max_foot_traveled, self.gait_state.stride_length )

    #==============================================================================
    def gait_update( self, leg, body_twist ):

        # Default to foot on the ground until we find otherwise
        self.leg_states[leg].foot_off_ground = False

        # Figure out if we are up or down
        if self.gait_state.gait_type == GaitStates.WAVE:
            # Hexapod wave gait has some weird edge cases. Check for them here
            if  (self.gt.phase >= self.leg_states[leg].phase_up and self.gt.phase < self.leg_states[leg].phase_down) or \
            (leg == 4-1 and (self.gt.phase >= self.leg_states[leg].phase_up or self.gt.phase < self.leg_states[leg].phase_down)):
                # Leg up ------------------------------
                self.leg_states[leg].foot_off_ground = True
                if leg == 4-1 and self.gt.phase < 1/NUM_LEGS:
                    self.leg_states[leg].lifting_phase = self.gt.phase * 3 + 0.5
                else:
                    self.leg_states[leg].lifting_phase = (self.gt.phase - self.leg_states[leg].phase_up) / (1 - self.gt.beta)

        # Check all other gait types
        elif self.gt.phase >= self.leg_states[leg].phase_up and self.gt.phase < self.leg_states[leg].phase_down:
            # Leg up ------------------------------
            self.leg_states[leg].foot_off_ground = True
            self.leg_states[leg].lifting_phase = (self.gt.phase - self.leg_states[leg].phase_up) / (1 - self.gt.beta)

        # Set new foot kinematics
        if self.leg_states[leg].foot_off_ground:
            self.integrate_foot_lift_pos( leg )
        else:
            self.integrate_foot_ground_pos( leg, body_twist )

        self.leg_states[leg].set_foot_center_dist( self.gait_state.foot_center[leg] )

    #==============================================================================
    def integrate_foot_ground_pos( self, leg, twist ):
        """ Uses commanded body twist vector, translate twist to desired foot current
        position, and uses calculated linear velocity and dt to integrate foot position
        Args:
            leg (int): Which leg to update foot position for. Expected range 0-5
            twist (geometry_msg/Twist): Commanded twist vector in body frame
        """
        # Current foot position
        p = np.array( [ self.leg_states[leg].foot_states.position.x,
                        self.leg_states[leg].foot_states.position.y,
                        0 ] ) # Force foot on the ground. May update later when foot sensors are implemented

        # Transformation from body twist to foot twist, all in the ground frame
        T_ground2foot = np.eye( 6 )
        T_ground2foot[3:,0:3] = rotation.skew( p )

        twist_body = np.array( [twist.angular.x, twist.angular.y, twist.angular.z,
                                -twist.linear.x, -twist.linear.y, -twist.linear.z] )
        twist_foot = T_ground2foot @ twist_body

        # Stash the calculated foot velocity
        self.leg_states[leg].set_foot_vel( twist_foot[3:])

        # Integrate foot position, ground frame
        foot_position = p + twist_foot[3:] * self.dt

        # Stash the calculated new position
        self.leg_states[leg].set_foot_pos( foot_position[:] )

    #==============================================================================
    def integrate_foot_lift_pos( self, leg ):

        if self.leg_states[leg].lifting_phase < self.leg_states[leg].previous_phase_up:
            # When we begin lifting our foot, stash our position before updating it
            self.leg_states[leg].set_phase_up_foot_start()

        self.leg_states[leg].previous_phase_up = self.leg_states[leg].lifting_phase

        # Calculate foot height z trajectory, with 0 starting and ending vel and accel
        z_pos = 0.0
        for coeff in range( len( self.gt.z_traj_coeff ) ):
            z_pos += self.gt.z_traj_coeff[coeff] * self.leg_states[leg].lifting_phase**coeff

        # Stash the position before updating it
        previous_pos = np.array([self.leg_states[leg].foot_states.position.x,
                                 self.leg_states[leg].foot_states.position.y,
                                 self.leg_states[leg].foot_states.position.z])

        x = np.interp( self.leg_states[leg].lifting_phase, [0,1.0], [self.leg_states[leg].phase_up_foot_start[0], self.gait_state.foot_center[leg].x] )
        y = np.interp( self.leg_states[leg].lifting_phase, [0,1.0], [self.leg_states[leg].phase_up_foot_start[1], self.gait_state.foot_center[leg].y] )
        z = self.gt.foot_height * z_pos
        self.leg_states[leg].set_foot_pos([x,y,z])

        twist_foot = np.zeros(6)
        twist_foot[3:] = (np.array([self.leg_states[leg].foot_states.position.x,
                                    self.leg_states[leg].foot_states.position.y,
                                    self.leg_states[leg].foot_states.position.z]) - previous_pos) / self.dt

        # Stash the calculated foot velocity
        self.leg_states[leg].set_foot_vel( twist_foot[3:] )

    #==============================================================================
    def get_gait_state( self, gait_state  ):
        self.gait_state = copy.deepcopy( gait_state )

#==============================================================================
if __name__ == "__main__":
    try:
        gu = gait_trajectory_update_node()
        gu.gait_trajectory_update()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

