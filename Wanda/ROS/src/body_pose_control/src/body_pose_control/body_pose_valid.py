#!/usr/bin/env python3
#=====================
import numpy as np
import rospy

# Standard ROS libraries
from geometry_msgs.msg import Transform
from tf.transformations import inverse_matrix
from tf.transformations import quaternion_matrix

# Custom libraries
from system_globals import transformation_frames

trans = transformation_frames.transformation_frames()

#==============================================================================
def body_pose_valid( pose, foot_center, workspace_min, workspace_max, links ):

    valid = True

    for leg in range( len( foot_center ) ):

        # Get 4x4 transformation matrix from ground to body given the pose
        T_g2b = quaternion_matrix( [ pose.rotation.x,
                                     pose.rotation.y,
                                     pose.rotation.z,
                                     pose.rotation.w ] ).T
        T_g2b[0:3,3] = np.array( [ pose.translation.x,
                                   pose.translation.y,
                                   pose.translation.z ] )

        valid = ground_cleared( leg, T_g2b )
        if not valid:
            # This pose collides with the ground. It's not valid.
            # No need to check other legs
            break

        valid = foot_workspace_reachable( leg,
                                          T_g2b,
                                          [ foot_center[leg].x,
                                            foot_center[leg].y,
                                            foot_center[leg].z ],
                                          workspace_min,
                                          workspace_max,
                                          links)
        if not valid:
            # This pose collides with the ground. It's not valid.
            # No need to check other legs
            break
    return valid

#==============================================================================
def ground_cleared( leg, T_g2b ):

    Tshoulder_belly = trans.T_body2shoulder[leg][:4,:4]
    Tshoulder_belly[2,3] = 0.0
    shoulder_gnd_frame = T_g2b @ Tshoulder_belly

    if shoulder_gnd_frame[2,3] < 0.0:
        # This shoulder is below the ground. We didn't clear
        # rospy.logwarn( f"Invalid body pose. Shoulder {leg} contacts the ground.")
        return False
    else:
        return True

#==============================================================================
def foot_workspace_reachable( leg, T_g2b, foot_center, workspace_min, workspace_max, links ):

    # Find the closest and farthest workspace points
    T_g2s = T_g2b @ trans.T_body2shoulder[leg]
    shoulder_pos_ground_frame = T_g2s[0:3,3]

    # Get the vector from the shoulder pointing to the workspace center
    vec_f2s = foot_center[:2] - shoulder_pos_ground_frame[:2]
    unit_vec_f2s = vec_f2s / np.linalg.norm( vec_f2s )

    ground_max_pos = foot_center[:2] + unit_vec_f2s * workspace_max
    ground_min_pos = foot_center[:2] - unit_vec_f2s * workspace_min

    # Figure out where the first link would place joint 2
    T_s2g = inverse_matrix( T_g2s )
    vec_s2f = T_s2g @ np.array( [ unit_vec_f2s[0],
                                       unit_vec_f2s[1],
                                       0.0,
                                       1.0 ] )

    unit_vec_s2f = vec_s2f / np.linalg.norm( vec_s2f )
    joint2 = np.array( [ unit_vec_s2f[0] * links[0], unit_vec_s2f[1] * links[0], 0.0 ] )

    # Find workspace min/max positions in shoulder frame
    ground_max_shoulder = T_s2g @ np.array( [ ground_max_pos[0],
                                              ground_max_pos[1],
                                              0.0,
                                              1.0 ] )

    reach = np.linalg.norm( ground_max_shoulder[0:3] - joint2[0:3] )
    if reach > links[1] + links[2]:
        # Outer edge of the workspace is too far for the leg to reach
        # rospy.logwarn( f"Invalid body pose. Leg {leg} cannot reach outer workspace limit.")
        return False

    # Make sure the body isn't going over top of the foot's workspace
    # ToDo. Note: I think, if we are smart with our ikine, we could in theory
    # handle the foot moving under the coxa. But that's tricky,
    # and this is a safer and easier assumption for now
    ground_min_shoulder = T_s2g @ np.array( [ ground_min_pos[0],
                                              ground_min_pos[1],
                                              0.0,
                                              1.0 ] )

    if np.linalg.norm( ground_min_shoulder[0:2] ) < links[0]:
        # rospy.logwarn( f"Invalid body pose. Leg {leg} workspace intersects first link workspace" )
        return False

    return True

#==============================================================================
if __name__ == "__main__":

    from body_pose_control import identity_pose

    pose = identity_pose.identity_pose()

    # Move the pose around to check it
    pose.translation.x = 0.0
    pose.translation.y = 0.0

    import rospy
    NUM_LEGS = rospy.get_param( 'num_legs', default=6 )
    L1 = rospy.get_param( 'L1', default=0.03226 )
    L2 = rospy.get_param( 'L2', default=0.090 )
    L3 = rospy.get_param( 'L3', default=0.113 )

    from walking_gait import init_gait_states
    gait_states = init_gait_states.init_gait_states( NUM_LEGS )

    valid = body_pose_valid( pose, gait_states.foot_center, gait_states.max_stride_length, [L1,L2,L3] )

    print(f"Body pose validity: {valid}")
