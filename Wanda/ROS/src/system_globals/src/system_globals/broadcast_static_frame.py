#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_matrix
import tf2_ros

tf2_broad = tf2_ros.StaticTransformBroadcaster()

#==============================================================================
def broadcast_static_frame( Ta2b, frame, child, time_stamp=None ):
    '''
    Given a 4x4 transformation matrix, a parent tf2 frame,
    and a child frame, broadcast the static transformation
    '''
    if not time_stamp:
        time_stamp = rospy.Time.now()

    # Set up header info
    transform = TransformStamped()
    transform.header.stamp = time_stamp
    transform.header.frame_id = frame
    transform.child_frame_id = child

    # Extract translation component
    transform.transform.translation.x = Ta2b[0,-1]
    transform.transform.translation.y = Ta2b[1,-1]
    transform.transform.translation.z = Ta2b[2,-1]

    # Extract rotation
    q = quaternion_from_matrix( Ta2b )
    transform.transform.rotation.x = q[0]
    transform.transform.rotation.y = q[1]
    transform.transform.rotation.z = q[2]
    transform.transform.rotation.w = q[3]

    # Send out transform
    tf2_broad.sendTransform( transform )
