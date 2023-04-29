#!/usr/bin/env python3
#=====================
import rospy
import tf2_ros

from system_globals import broadcast_static_frame
from system_globals import frames
from system_globals import transformation_frames

seeker_broadcaster = tf2_ros.TransformBroadcaster()
frame = frames.frames()
trans = transformation_frames.transformation_frames()

#==============================================================================
def broadcast_seeker_fkine( joint_angles, time_stamp=None ):
    '''Given array of 2 joint angles, broadcast the seeker tf frame'''

    if not time_stamp:
        time_stamp = rospy.Time.now()

    broadcast_static_frame.broadcast_static_frame( trans.T_body2neck_static, frame.body, frame.neck_static, time_stamp )

    # Send transform from azimuth to elevation joint
    T_neck_az2el = trans.transform_neck_static2neck( joint_angles[0] )
    seeker_tf = trans.trans_matrix2topic( T_neck_az2el, time_stamp, frame.neck_static, frame.neck )
    seeker_broadcaster.sendTransform( seeker_tf )

    # Send transform from elevation joint to seeker
    T_neck_el2seeker = trans.transform_neck2seeker( joint_angles[1] )
    seeker_tf = trans.trans_matrix2topic( T_neck_el2seeker, time_stamp, frame.neck, frame.seeker )
    seeker_broadcaster.sendTransform( seeker_tf )

    broadcast_static_frame.broadcast_static_frame( trans.T_seeker2cam, frame.seeker, frame.camera, time_stamp )

    broadcast_static_frame.broadcast_static_frame( trans.T_seeker2ultra, frame.seeker, frame.ultrasonic, time_stamp )
