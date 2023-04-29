#!/usr/bin/env python3
#=====================
import copy
import numpy as np
import rospy

from geometry_msgs.msg import Point

from system_globals import frames
from walking_gait.msg import GaitStates

frame = frames.frames()

#============================================================
def init_gait_states( num_legs = 6):
    gait_state = GaitStates()
    gait_state.gait_type = GaitStates.WAVE # For now, just leave it as a wave gait. Future logic will set this dynamically
    gait_state.header.frame_id = frame.ground
    gait_state.max_stride_length = rospy.get_param( 'gait_state/max_stride_length', default=0.08 )
    gait_state.min_stride_length = gait_state.max_stride_length / 2
    gait_state.stride_length = copy.deepcopy( gait_state.max_stride_length )

    foot_center = np.reshape( rospy.get_param( 'gait_state/foot_center_init' ), (3,num_legs) )
    for leg in range( num_legs ):
        foot_center_point = Point()
        foot_center_point.x = foot_center[0,leg]
        foot_center_point.y = foot_center[1,leg]
        foot_center_point.z = foot_center[2,leg]
        gait_state.foot_center.append( foot_center_point )

    return gait_state