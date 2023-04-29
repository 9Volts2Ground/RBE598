#!/usr/bin/env python3
#=====================
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import TransformStamped

from body_pose_control.identity_pose import identity_pose
from walking_gait.msg import FootStates

#==============================================================================
class leg_states():
    def __init__(self,
                 leg=0,
                 init_pos=Point(),
                 foot_frame=f"foot_ground0",
                 ground_frame="ground"):

        self.leg = leg

        # Gait-specific parameters
        self.phase_up = 0.0 # What point in the phase the foot lifts
        self.phase_down = 0.0 # What point in the phase the foot contacts the ground again

        self.foot_center_dist = 0.0
        self.foot_center_dist_delta = 0.0

        # Position where we start the up phase, so we can interpolate
        self.phase_up_foot_start = np.zeros( 3 )
        self.previous_phase_up = 1.0

        # For the stride portion of the gait, what is our phase then?
        self.lifting_phase = 0.0

        self.foot_off_ground = False

        # Initialize foot states
        self.foot_states = FootStates()
        self.foot_states.header.frame_id = foot_frame
        self.foot_states.position.x = init_pos.x
        self.foot_states.position.y = init_pos.y
        self.foot_states.position.z = init_pos.z
        self.foot_states.velocity.linear.x = 0.0
        self.foot_states.velocity.linear.y = 0.0
        self.foot_states.velocity.linear.z = 0.0

        # Initialize transform
        self.foot_trans = TransformStamped()
        self.foot_trans.header.frame_id = ground_frame
        self.foot_trans.child_frame_id = foot_frame
        self.foot_trans.transform = identity_pose()
        self.foot_trans.transform.translation.x = init_pos.x
        self.foot_trans.transform.translation.y = init_pos.y
        self.foot_trans.transform.translation.z = init_pos.z

    #================================================================
    def set_foot_pos( self, pos ):
        self.foot_states.position.x = pos[0]
        self.foot_states.position.y = pos[1]
        self.foot_states.position.z = pos[2]

        self.foot_trans.transform.translation.x = pos[0]
        self.foot_trans.transform.translation.y = pos[1]
        self.foot_trans.transform.translation.z = pos[2]

    #================================================================
    def set_foot_vel( self, vel ):
        self.foot_states.velocity.linear.x = vel[0]
        self.foot_states.velocity.linear.y = vel[1]
        self.foot_states.velocity.linear.z = vel[2]

    #================================================================
    def set_foot_center_dist( self, foot_center ):

        foot_to_center = [foot_center.x - self.foot_states.position.x,
                          foot_center.y - self.foot_states.position.y,
                          foot_center.z - self.foot_states.position.z]

        foot_center_dist = np.linalg.norm( foot_to_center )

        # Store how far it changed since last time
        self.foot_center_dist_delta = foot_center_dist - self.foot_center_dist
        self.foot_center_dist = foot_center_dist

    #================================================================
    def set_foot_phase( self, phase_offset, beta ):
        self.phase_up = phase_offset
        self.phase_down = self.phase_up + (1 - beta)
        while self.phase_down > 1:
            self.phase_down -= 1

    #================================================================
    def set_phase_up_foot_start( self ):
        self.phase_up_foot_start = [self.foot_states.position.x,
                                    self.foot_states.position.y,
                                    self.foot_states.position.z]
