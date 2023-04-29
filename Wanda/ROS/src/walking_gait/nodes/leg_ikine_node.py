#!/usr/bin/env python3
#=====================
import copy
import numpy as np
import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import tf2_ros
from tf.transformations import inverse_matrix
from tf.transformations import quaternion_matrix

from system_globals import frames
from system_globals.jacobian import jacobian
from system_globals import transformation_frames
from topic_messages import topics
from walking_gait.msg import FootStates
from walking_gait import rotation

# Globals
frame = frames.frames()
top = topics.topics()
trans = transformation_frames.transformation_frames()
PI = np.pi
HALF_PI = PI / 2

#============================================================
# Grab global parameters
#============================================================
NUM_LEGS = rospy.get_param( 'num_legs', default=6 )
L1 = rospy.get_param( 'L1', default=0.03226 )
L2 = rospy.get_param( 'L2', default=0.090 )
L3 = rospy.get_param( 'L3', default=0.113 )
SERVO_MAX_VEL = rospy.get_param( 'leg_servos/max_rate', default=2*np.pi)

#==============================================================================
class leg_ikine_node():
    def __init__(self):
        rospy.init_node( "leg_ikine_node", anonymous=True )

        # Initializes body pose to home position
        self.Tbg = np.eye( 4 )

        # Listen to the body transform
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener( self.tfBuffer )

        # Publish new joint states any time the feet update
        foot_center = np.reshape( rospy.get_param( 'gait_state/foot_center_init' ), (3,NUM_LEGS) )
        self.joint_states = []
        self.pub = []
        for leg in range( NUM_LEGS ):
            self.joint_states.append( JointState() )
            self.joint_states[leg].header.frame_id = frame.shoulder[leg]
            self.joint_states[leg].name = ["hip", "knee", "ankle"]

            self.joint_states[leg].position = np.zeros(3)
            self.joint_states[leg].position = self.foot_position_to_joint_angles( foot_center[:,leg], self.joint_states[leg].position, leg )
            self.joint_states[leg].velocity = np.zeros(3)
            self.joint_states[leg].effort = np.zeros(3)

            self.pub.append( rospy.Publisher( top.leg_joint_states[leg], JointState, queue_size=5 ) )
            rospy.Subscriber( top.leg_foot_states[leg], FootStates, self.leg_ikine, leg )

    #==============================================================================
    def leg_ikine( self, foot_states, leg ):

        self.get_ground_body_transform()

        # Convert foot states from the ground frame to the shoulder frame
        foot_vec_ground = np.array( [ foot_states.position.x,
                                      foot_states.position.y,
                                      foot_states.position.z,
                                      1.0 ] )
        Tsg = inverse_matrix( trans.T_body2shoulder[leg] ) @ self.Tbg

        # Get our joint angles
        foot_vec_shoulder = Tsg @ foot_vec_ground.T
        self.joint_states[leg].position = self.foot_position_to_joint_angles( foot_vec_shoulder,
                                                                              self.joint_states[leg].position,
                                                                              leg )

        # Get joint velocities
        Ttwist_s2g = rotation.skew( Tsg[0:3,3] ) @ Tsg[0:3,0:3]

        foot_velocity_shoulder = Ttwist_s2g @ np.array( [ foot_states.velocity.linear.x,
                                                          foot_states.velocity.linear.y,
                                                          foot_states.velocity.linear.z ] ).T
        self.joint_states[leg].velocity = self.calculate_joint_velocities( self.joint_states[leg].position,
                                                                           foot_velocity_shoulder,
                                                                           leg )

        # Package up states for publishing
        self.joint_states[leg].header.stamp = rospy.Time.now()
        self.pub[leg].publish( self.joint_states[leg] )

    #==============================================================================
    def foot_position_to_joint_angles( self, pos, joint_angles_old, leg ):
        """Given array of [x,y,z] foot coordinates in the leg shoulder frame,
        calculate shoulder, knee, and ankle joint angles necessary to achieve
        the desired pose

        Args:
            pos[3] (float): array of x,y,z foot position in shoulder frame, meters

        Returns:
            joint_angles[3] (float): shoulder, knee, ankle joint angles, radians
        """

        joint_angles = joint_angles_old[:3] # Initialize array

        joint_angles[0] = np.arctan2( pos[1], pos[0] )

        # If the desired foot position is behind L1, keep L1 pointing out away from the body
        if joint_angles[0] > HALF_PI:
            joint_angles[0] -=  PI
        elif joint_angles[0] < -HALF_PI:
            joint_angles[0] += PI

        # Length of base of triangle from q2 to end effector
        r = np.linalg.norm( np.array( pos[0:2] ) ) - L1

        if r > L2 + L3:
            rospy.logerr(f"Desired ikine position for leg {leg} out of reach: {pos[:3]}")
            # For now, just return the old q2 and q3 angles, since new new position can't be reached
            return joint_angles

        try:
            D = ( r**2 + pos[2]**2 - L2**2 - L3**2 ) / ( 2 * L2 * L3 )
            joint_angles[2] = np.arctan2( np.sqrt( 1 - D**2 ), D )
        except:
            rospy.logerr(f"Could not compute 3rd leg{leg} joint angle. Using old angle {joint_angles[2]}")

        gamma = np.arctan2( -pos[2], r )
        beta = np.arctan2( L3 * np.sin( joint_angles[2] ), \
            L2 + L3*np.cos( joint_angles[2] ) )

        joint_angles[1] = -( gamma - beta ) # Negative causes knee-up configuration

        return joint_angles

    #==============================================================================
    def calculate_joint_velocities( self, joint_angles, foot_velocity, leg ):

        J = jacobian( joint_angles[:3] )
        detJ = np.linalg.det( J )
        if abs( detJ ) < 0.0001:
            rospy.logwarn(f"det(Jacobian)={detJ} is too small for leg {leg}")

        # Sometimes pinv(J) does not converge. Set inside try/except case to prevent it from crashing at runt ime
        joint_velocity = np.zeros(3)
        try:
            J_t = np.linalg.pinv( J )
            joint_velocity = J_t @ foot_velocity[:3].T
            if any( abs( joint_velocity ) > SERVO_MAX_VEL ):
                rospy.logwarn(f"Leg {leg} joint vel exceed servo specs: {joint_velocity}. Foot pos may not be reached in desired timed")
        except:
            rospy.logerr(f"pinv(J) failed to set joint velocities for leg {leg}")

        return joint_velocity

    #==============================================================================
    def get_ground_body_transform( self ):
        """Grab ground_body_transform transform, store the transformation matrix
        """

        try:
            tran = self.tfBuffer.lookup_transform( frame.body, frame.ground, rospy.Time() )

            q = [tran.transform.rotation.x,
                 tran.transform.rotation.y,
                 tran.transform.rotation.z,
                 tran.transform.rotation.w]

            self.Tbg = np.array( quaternion_matrix( q ) )
            self.Tbg[:3,3] = [tran.transform.translation.x,
                              tran.transform.translation.y,
                              tran.transform.translation.z]
        except:
            rospy.logwarn_throttle( 10, f"Could not get T_body2ground in {rospy.get_name()}. Using previous transformation")

#==============================================================================
if __name__ == "__main__":
    try:
        ikine = leg_ikine_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
