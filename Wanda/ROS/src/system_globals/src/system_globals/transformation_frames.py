#!/usr/bin/env python3
import numpy as np

from geometry_msgs.msg import TransformStamped
import rospy
from tf.transformations import quaternion_from_matrix

# Grab system parameters
NUM_LEGS = rospy.get_param( 'num_legs', default=6 )
L1 = rospy.get_param( 'L1', default=0.03226 )
L2 = rospy.get_param( 'L2', default=0.090 )
L3 = rospy.get_param( 'L3', default=0.113 )
S_VEC = np.reshape( rospy.get_param( 's_vec' ), (3,NUM_LEGS) )
ALPHA_OFFSET = rospy.get_param( 'alpha_offset' )
BODY2NECK_AZ = rospy.get_param( 'body2neck_az', default=[0.083076, 0.0, 0.08995] )
NECK_AZ2EL = rospy.get_param( 'neck_az2el', default=[0.0104, 0.0, 0.04121] )
IMU_POS = rospy.get_param( 'imu_pos', default=[0.04502, 0.02, 0.088951] )
SEEKER_CAM_OFFSET = rospy.get_param( 'seeker_cam_offset', default=[0.0, 0.0, 0.0] )
SEEKER_ULTRA_OFFSET = rospy.get_param( 'seeker_ultra_offset', default=[0.0, 0.0, 0.0] )

class transformation_frames:
    def __init__( self ):
        #==============================================================
        # Constant transforms for the robot
        #==============================================================
        # Transformation from body frame to el motor axis
        self.T_body2neck_static = np.array( [ [1.0, 0.0, 0.0, BODY2NECK_AZ[0]],
                                              [0.0, 1.0, 0.0, BODY2NECK_AZ[1]],
                                              [0.0, 0.0, 1.0, BODY2NECK_AZ[2]],
                                              [0.0, 0.0, 0.0,             1.0] ] )

        self.T_seeker2cam = np.array( [ [0.0,  0.0, 1.0, SEEKER_CAM_OFFSET[0]],
                                        [0.0, -1.0, 0.0, SEEKER_CAM_OFFSET[1]],
                                        [1.0,  0.0, 0.0, SEEKER_CAM_OFFSET[2]],
                                        [0.0,  0.0, 0.0,                  1.0] ] )

        self.T_seeker2ultra = np.array( [ [1.0, 0.0, 0.0, SEEKER_ULTRA_OFFSET[0]],
                                          [0.0, 1.0, 0.0, SEEKER_ULTRA_OFFSET[1]],
                                          [0.0, 0.0, 1.0, SEEKER_ULTRA_OFFSET[2]],
                                          [0.0, 0.0, 0.0,                    1.0] ] )

        self.T_body2imu = np.array( [ [1.0, 0.0, 0.0, IMU_POS[0]],
                                      [0.0, 1.0, 0.0, IMU_POS[1]],
                                      [0.0, 0.0, 1.0, IMU_POS[2]],
                                      [0.0, 0.0, 0.0,        1.0] ] )

        self.T_body2shoulder = []
        for leg in range( NUM_LEGS ):
            self.T_body2shoulder.append( self.transform_body2shoulder( leg ) )

    #==========================================================================
    def transform_body2shoulder( self, leg ):
        """
        Args:
            leg (integer): Which leg to get transformation for
        Returns:
            T_body2shoulder [4x4 np array]: Transformation from body frame to shoulder joint of leg
        """

        c_alpha = np.cos( ALPHA_OFFSET[leg] )
        s_alpha = np.sin( ALPHA_OFFSET[leg] )

        T_body2shoulder = np.array( [[ c_alpha, -s_alpha, 0.0, S_VEC[0,leg] ],
                                     [ s_alpha,  c_alpha, 0.0, S_VEC[1,leg] ],
                                     [     0.0,      0.0, 1.0, S_VEC[2,leg] ],
                                     [     0.0,      0.0, 0.0,          1.0 ] ] )
        return T_body2shoulder

    #==========================================================================
    def transform_shoulder2coxa( self, angle ):
        """
        Args:
            leg (integer): Which leg to get transformation for
        Returns:
            T_shoulder2coxa [4x4 np array]: Transformation from body frame to hip joint of leg
        """

        c_hip = np.cos( angle )
        s_hip = np.sin( angle )

        T_shoulder2coxa = np.array( [ [ c_hip, -s_hip, 0.0, 0.0 ],
                                     [ s_hip,  c_hip, 0.0, 0.0 ],
                                     [   0.0,    0.0, 1.0, 0.0 ],
                                     [   0.0,    0.0, 0.0, 1.0 ] ] )
        return T_shoulder2coxa

    #==========================================================================
    def transform_coxa2femur( self, angle, link_length = L1 ):
        """Gets transformation matrix from shoulder joint to knee joint.
        Option to change link length, like to use link CG instead of full length.
        Defaults to L1
        Args:
            angle (float): Joint angle of shoulder joint (rad)
            link_length (float, optional): Length of leg link. Defaults to L1.
        Returns:
            4x4 numpy array: Transformation matrix
        """
        c_knee = np.cos( angle )
        s_knee = np.sin( angle )

        T_coxa2femur = np.array( [ [ c_knee, -s_knee,  0.0, link_length ],
                                   [    0.0,     0.0, -1.0,         0.0 ],
                                   [ s_knee,  c_knee,  0.0,         0.0 ],
                                   [    0.0,     0.0,  0.0,         1.0 ] ] )
        return T_coxa2femur

    #==========================================================================
    def transform_femur2tibia( self, angle, link_length = L2 ):
        """Gets transformation matrix from knee joint to ankle joint.
        Option to change link length, like to use link CG instead of full length.
        Defaults to L2
        Args:
            angle (float): Joint angle of knee joint (rad)
            link_length (float, optional): Length of leg link. Defaults to L2.
        Returns:
            4x4 numpy array: Transformation matrix
        """
        c_knee = np.cos( angle )
        s_knee = np.sin( angle )

        return np.array( [ [  c_knee, -s_knee,  0.0, link_length ],
                           [ -s_knee, -c_knee,  0.0,         0.0 ],
                           [     0.0,     0.0, -1.0,         0.0 ],
                           [     0.0,     0.0,  0.0,         1.0 ] ] )

    #==========================================================================
    def transform_tibia2foot( self, link_length = L3 ):
        """Gets transformation matrix from ankle joint to foot.
        Option to change link length, like to use link CG instead of full length.
        Defaults to L3
        Args:
            angle (float): Joint angle of ankle joint (rad)
            link_length (float, optional): Length of leg link. Defaults to L3.
        Returns:
            4x4 numpy array: Transformation matrix
        """

        return np.array( [ [ 1.0, 0.0, 0.0, link_length ],
                           [ 0.0, 1.0, 0.0,         0.0 ],
                           [ 0.0, 0.0, 1.0,         0.0 ],
                           [ 0.0, 0.0, 0.0,         1.0 ] ] )

    #==========================================================================
    def transform_neck_static2neck( self, az_angle ):
        """Gets transformation matrix from neck azimuth to elevation axes.
        Args:
            neck_az2el (float): Joint angle of neck azimuth joint (rad)
            link_length (float, optional): Length of leg link. Defaults to L3.
        Returns:
            4x4 numpy array: Transformation matrix
        """
        c_az = np.cos( az_angle )
        s_az = np.sin( az_angle )

        return np.array( [ [ c_az, -s_az, 0.0, 0.0 ],
                           [ s_az,  c_az, 0.0, 0.0 ],
                           [  0.0,   0.0, 1.0, 0.0 ],
                           [  0.0,   0.0, 0.0, 1.0 ] ] )

    #==========================================================================
    def transform_neck2seeker( self, el_angle, neck = NECK_AZ2EL ):
        # Positive az turns clockwise, positive el turns seeker up
        # Precompute sin/cos values for efficiency
        cos_el = np.cos( el_angle )
        sin_el = np.sin( el_angle )

        # Single matrix from neck to camera
        T_neck_el2seeker = np.array( [ [ cos_el, -sin_el,  0.0, neck[0] ],
                                       [    0.0,     0.0, -1.0,     0.0 ],
                                       [ sin_el,  cos_el,  0.0, neck[2] ],
                                       [    0.0,     0.0,  0.0,     1.0 ] ] )
        return T_neck_el2seeker

    #==========================================================================
    def trans_matrix2topic( self, T_a2b, time_stamp, frame='', child='' ):
        # Package and send transform
        tran = TransformStamped()
        tran.header.stamp = time_stamp
        if frame:
            tran.header.frame_id = frame
        if child:
            tran.child_frame_id = child

        tran.transform.translation.x = T_a2b[0,3]
        tran.transform.translation.y = T_a2b[1,3]
        tran.transform.translation.z = T_a2b[2,3]

        q = quaternion_from_matrix( T_a2b )
        tran.transform.rotation.x = q[0]
        tran.transform.rotation.y = q[1]
        tran.transform.rotation.z = q[2]
        tran.transform.rotation.w = q[3]

        return tran

#==============================================================================
if __name__ == "__main__":
    tran = transformation_frames()

    print("done")
