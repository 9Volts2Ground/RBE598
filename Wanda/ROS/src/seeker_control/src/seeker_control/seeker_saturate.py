#!/usr/bin/env python3
#=====================
import numpy as np
import rospy

#============================================================
# Grab global parameters
#============================================================
NUM_SEEKER_JOINTS  = rospy.get_param( 'num_seeker_joints', default=2 )
SEEKER_CENTER      = rospy.get_param( 'seeker_servos/center', default=[85, 91] )
SEEKER_MIN         = rospy.get_param( 'seeker_servos/min', default=[20, 75] )
SEEKER_MAX         = rospy.get_param( 'seeker_servos/max', default=[160, 180])
SEEKER_ORIENTATION = rospy.get_param( 'seeker_servos/orientation', default=[1, 1])

seeker_max_rad = [0,0]
seeker_min_rad = [0,0]
for joint in range( NUM_SEEKER_JOINTS ):
    seeker_max_rad[joint] = np.deg2rad( SEEKER_MAX[joint] - SEEKER_CENTER[joint] ) * SEEKER_ORIENTATION[joint]
    seeker_min_rad[joint] = np.deg2rad( SEEKER_MIN[joint] - SEEKER_CENTER[joint] ) * SEEKER_ORIENTATION[joint]

#================================================================
def seeker_saturate( joint_angles ):

    saturated = [False, False]
    for joint in range( NUM_SEEKER_JOINTS ):

        if joint_angles[joint] > seeker_max_rad[joint]:
            joint_angles[joint] = seeker_max_rad[joint]
            saturated[joint] = True
        elif joint_angles[joint] < seeker_min_rad[joint]:
            joint_angles[joint] = seeker_min_rad[joint]
            saturated[joint] = True

    return joint_angles, saturated

#==============================================================================
if __name__ == "__main__":
    test_angles = [np.pi/2,np.pi/2]

    returned =  seeker_saturate( test_angles )

