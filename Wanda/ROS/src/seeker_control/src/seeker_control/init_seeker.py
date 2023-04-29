#!/usr/bin/env python3
#=====================
from sensor_msgs.msg import JointState

# Custom packages
from system_globals import frames

frame = frames.frames()

#================================================
def init_seeker( joint_pos = [0.0, 0.0],
                 joint_vel = [0.0, 0.0],
                 neck_frame = frame.neck_static ):
    # Initialize seeker state topic
    seeker_state = JointState()
    seeker_state.header.frame_id = neck_frame
    seeker_state.name = ["azimuth", "elevation"]
    seeker_state.position = [ joint_pos[0], joint_pos[1] ]
    seeker_state.velocity = [ joint_vel[0], joint_vel[1] ]
    seeker_state.effort = [ 0.0, 0.0 ]

    return seeker_state
