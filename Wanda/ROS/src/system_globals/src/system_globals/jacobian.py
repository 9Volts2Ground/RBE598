#!/usr/bin/env python3
#=====================
import numpy as np
import rospy

#============================================================
# Grab global parameters
#============================================================
L1 = rospy.get_param( 'L1', default=0.03226 )
L2 = rospy.get_param( 'L2', default=0.090 )
L3 = rospy.get_param( 'L3', default=0.113 )

#==============================================================================
def jacobian( joint_angles ):
    '''Calculates the Jacobian with respect to the shoulder frame
    '''

    c1 = np.cos( joint_angles[0] )
    s1 = np.sin( joint_angles[0] )
    c2 = np.cos( joint_angles[1] )
    s2 = np.sin( joint_angles[1] )
    c32 = np.cos( joint_angles[2]- joint_angles[1] )
    s32 = np.sin( joint_angles[2]- joint_angles[1] )

    J = np.array( [ [-s1*(L1 + L2*c2 + L3*c32), -c1*(L2*s2 - L3*s32), -L3*c1*s32],
                    [ c1*(L1 + L2*c2 + L3*c32), -s1*(L2*s2 - L3*s32), -L3*s1*s32],
                    [                        0,       L2*c2 + L3*c32,    -L3*c32] ] )

    return J

#==============================================================================
if __name__ == "__main__":

    joint_angles = [np.pi/8,np.pi/6,0]
    J = jacobian( joint_angles )
    print(J)