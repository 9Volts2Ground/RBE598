#!/usr/bin/env python3
#=====================
import copy
import numpy as np

# ROS libraries
import actionlib
import rospy
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf.transformations import inverse_matrix
from tf.transformations import quaternion_matrix

# Custom libraries
from body_pose_control.identity_pose import identity_pose
from system_globals import frames
from system_globals import transformation_frames
from topic_messages import topics
from walking_gait.msg import GaitStates
from walking_gait.msg import IncrementFootCenterAction
from walking_gait.init_gait_states import init_gait_states

#============================================================
# Grab global hardware parameters
#============================================================
NUM_LEGS = rospy.get_param( 'num_legs', default=6 )
L1 = rospy.get_param( 'L1', default=0.03226 )
L2 = rospy.get_param( 'L2', default=0.090 )
L3 = rospy.get_param( 'L3', default=0.113 )

frame = frames.frames()
top = topics.topics()
trans = transformation_frames.transformation_frames()

#==============================================================================
class gait_state_control_node():
    def __init__(self):
        rospy.init_node( "gait_state_control", anonymous=True )

        self.singularity_safety_scale = 0.9

        # Pre-compute some constants to save processing
        self.knee_joint_body = np.zeros( (3, NUM_LEGS) )
        self.mag_knee_joint_body = np.zeros( NUM_LEGS )
        self.full_leg = np.zeros( (3, NUM_LEGS) )
        self.mag_full_leg = np.zeros( NUM_LEGS )
        for leg in range( NUM_LEGS ):
            knee_joint_body = trans.T_body2shoulder[0] @ np.array( [L1, 0.0, 0.0, 1.0 ] ).T
            self.knee_joint_body[:,leg] = knee_joint_body[:3]
            self.mag_knee_joint_body[leg] = np.linalg.norm( self.knee_joint_body[:,leg] )

            full_leg = trans.T_body2shoulder[0] @ np.array( [L1+L2+L3, 0.0, 0.0, 1.0 ] ).T
            self.full_leg[:,leg] = full_leg[:3]
            self.mag_full_leg[leg] = np.linalg.norm( self.full_leg[:,leg] )

        # Set up our copy of the walking gait states
        self.gait_state = init_gait_states( NUM_LEGS )
        self.foot_center = np.zeros( (3,6 ) )
        self.foot_center_updated = False
        self.set_foot_center_array()
        self.set_stride_length_bounds()
        self.gait_state.stride_length = copy.deepcopy( self.gait_state.max_stride_length )

        # Initialize body pose
        self.pose = TransformStamped()
        self.pose.header.frame_id = frame.ground
        self.pose.child_frame_id = frame.body
        self.pose.transform = identity_pose()

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener( self.tfBuffer )

        # Set up publisher
        self.pub = rospy.Publisher( top.gait_state, GaitStates, queue_size=1 )

        # Set up server to update feet positions
        self.server = actionlib.SimpleActionServer( 'foot_center_server',
                                                    IncrementFootCenterAction,
                                                    self.increment_foot_center,
                                                    auto_start=False )
        self.server.start()

        # Set up publisher loop
        self.loop_hz = 30
        self.rate = rospy.Rate( self.loop_hz )
        self.publish_gait_state()


    #==========================================================================
    def publish_gait_state( self ):

        # Loop through and send out our updated gait parameters
        while not rospy.is_shutdown():

            tran = self.get_ground_body_transform()
            self.update_stride_limits( tran )

            self.gait_state.header.stamp = rospy.Time.now()

            self.pub.publish( self.gait_state )
            self.rate.sleep()

    #==============================================================================
    def get_ground_body_transform( self ):
        """Grab ground_body_transform transform, store the transformation matrix
        """
        try:
            tran = self.tfBuffer.lookup_transform( frame.ground, frame.body, rospy.Time() )
            return tran
        except:
            rospy.logwarn_throttle( 10, f"Could not get T_ground2body in {rospy.get_name()}. Using previous transformation")
            return self.pose

    #==========================================================================
    def update_stride_limits( self, pose ):

        if self.pose.transform != pose.transform or self.foot_center_updated:

            self.foot_center_updated = False

            # Get 4x4 transformation matrix from ground to body given the pose
            T_g2b = quaternion_matrix( [ pose.transform.rotation.x,
                                        pose.transform.rotation.y,
                                        pose.transform.rotation.z,
                                        pose.transform.rotation.w ] ).T
            T_g2b[0:3,3] = np.array( [ pose.transform.translation.x,
                                    pose.transform.translation.y,
                                    pose.transform.translation.z ] )

            # Reset the stride length to its max, then shrink it if need be
            self.gait_state.stride_length = copy.deepcopy( self.gait_state.max_stride_length )

            for leg in range( NUM_LEGS ):
                # Find the closest and farthest workspace points
                T_g2s = T_g2b @ trans.T_body2shoulder[leg]
                shoulder_pos_ground_frame = T_g2s[:3,3]

                # Get the vector from the shoulder pointing to the workspace center
                vec_f2s = self.foot_center[:2,leg] - shoulder_pos_ground_frame[:2]
                unit_vec_f2s = vec_f2s / np.linalg.norm( vec_f2s[:2] )

                max_stride_ground_vec = unit_vec_f2s[:2] * self.gait_state.max_stride_length
                ground_max_pos = self.foot_center[:2,leg] + max_stride_ground_vec
                ground_min_pos = self.foot_center[:2,leg] - max_stride_ground_vec

                # Figure out where the first link would place joint 2
                T_s2g = inverse_matrix( T_g2s )
                vec_s2f = T_s2g @ np.array( [ unit_vec_f2s[0],
                                              unit_vec_f2s[1],
                                              0.0,
                                              1.0 ] )

                # Vector to the end of the coxa in the shoulder frame
                unit_vec_s2f = vec_s2f / np.linalg.norm( vec_s2f )
                joint2 = np.array( [ unit_vec_s2f[0]*L1, unit_vec_s2f[1]*L1, 0.0, 1.0 ] )

                # Check that the leg can reach the max position
                ground_max_shoulder = T_s2g @ np.array( [ground_max_pos[0], ground_max_pos[1], 0.0, 1.0 ] ).T

                max_reach = np.linalg.norm( ground_max_shoulder[:3] - joint2[:3] )
                L23 = L2 + L3
                joint2_ground = T_g2s @ joint2

                if max_reach > L23:
                    # We can't reach the edge of the workspace. Gotta shrink it
                    # Find the triangle between the outer workspace, the knee joint, and the ground under the knee
                    stride_length = np.sqrt( L23**2 - joint2_ground[2]**2 ) - np.linalg.norm( self.foot_center[:2,leg] - joint2_ground[:2] )

                    if stride_length < self.gait_state.stride_length:
                        # rospy.logwarn( f"Leg {leg} outer workspace exceeded. Updating stride length = {stride_length}")
                        if stride_length > self.gait_state.min_stride_length:
                            self.gait_state.stride_length = copy.deepcopy( stride_length )
                        else:
                            self.gait_state.stride_length = copy.deepcopy( self.gait_state.min_stride_length )

                # Now make sure the pose hasn't made the shoulder encroach on the workspace
                ground_min_shoulder = T_s2g @ np.array( [ground_min_pos[0], ground_min_pos[1], 0.0, 1.0 ] ).T
                if np.linalg.norm( ground_min_shoulder[:2] - joint2[:2] ) < L1:
                    # The shoulder is above the max workspace. Shrink the stride length down

                    knee_z = T_g2s @ np.array( [0, 0, 1, 1] ) # Note: this isn't necessarily a unit vector

                    # Angle between the knee verticle and the ground plane
                    theta = np.arccos( np.dot( knee_z[:3], np.array( [0, 0, 1] ) ) / np.linalg.norm( knee_z[:3]) )

                    stride_length = np.linalg.norm( self.foot_center[:2,leg] - joint2_ground[:2] ) - joint2_ground[2] * np.tan( theta )

                    if stride_length < self.gait_state.stride_length:
                        # rospy.logwarn( f"Leg {leg} outer workspace exceeded. Updating stride length = {stride_length}")
                        if stride_length > self.gait_state.min_stride_length:
                            self.gait_state.stride_length = copy.deepcopy( stride_length )
                        else:
                            self.gait_state.stride_length = copy.deepcopy( self.gait_state.min_stride_length )


        # If we are still happy with the new pose, accept it
        self.pose = copy.deepcopy( pose )

    #==========================================================================
    def set_stride_length_bounds( self ):

        # Check distance between foot centers to ensure they don't conflict
        # We can just check one side of the robot since it should be symmetric
        self.gait_state.max_stride_length = np.linalg.norm( self.foot_center[:,0] - self.foot_center[:,2] )
        foot2_foot4_dist = np.linalg.norm( self.foot_center[:,2] - self.foot_center[:,4] )
        if foot2_foot4_dist < self.gait_state.max_stride_length:
            self.gait_state.max_stride_length = foot2_foot4_dist

        # Now check the distance from the foot home to the first knee joints at their closest point
        foot0_knee0_dist = np.linalg.norm( self.knee_joint_body[:2,0] - self.foot_center[:2,0] )
        if foot0_knee0_dist < self.gait_state.max_stride_length:
            self.gait_state.max_stride_length = foot0_knee0_dist

        # Scale it down a bit for some safety factor
        self.gait_state.max_stride_length *= self.singularity_safety_scale
        self.gait_state.min_stride_length = self.gait_state.max_stride_length / 2

    #==========================================================================
    def set_foot_center_array( self ):
        ''' Creates a numpy array from the foot center points, to aid in computation'''
        for leg in range( NUM_LEGS ):
            self.foot_center[:,leg] = [self.gait_state.foot_center[leg].x,
                                       self.gait_state.foot_center[leg].y,
                                       self.gait_state.foot_center[leg].z]

    #==========================================================================
    def update_foot_center( self ):
        ''' Shoves numpy array foot center into gait_states point '''
        for leg in range( NUM_LEGS ):
            self.gait_state.foot_center[leg].x = self.foot_center[0,leg]
            self.gait_state.foot_center[leg].y = self.foot_center[1,leg]
            self.gait_state.foot_center[leg].z = self.foot_center[2,leg]

    #==========================================================================
    def increment_foot_center( self, goal ):

        if goal.increment_mag == 0.0:
            # We're not actually changing anything. Don't bother
            self.server.set_aborted()
            return

        new_foot_center = np.zeros( (3, NUM_LEGS) )
        for leg in range( NUM_LEGS ):

            foot_center_dist = np.linalg.norm( self.foot_center[:,leg] )
            foot_center_uv = self.foot_center[:,leg] / foot_center_dist
            new_foot_center_dist = foot_center_dist + goal.increment_mag

            # Make sure it's valid
            if new_foot_center_dist < self.mag_knee_joint_body[leg] * 1.2:
                # The foot is trying to get too close. Don't accept it
                self.server.set_aborted()
                return

            if new_foot_center_dist > self.mag_full_leg[leg] * 0.7:
                # This foot is reaching out way too far. Not acceptable
                self.server.set_aborted()
                return

            # It looks good, Keep it
            new_foot_center[:,leg] = foot_center_uv * new_foot_center_dist

        # Looks like none of the legs rejected the position. Keep it
        for leg in range( NUM_LEGS ):
            self.foot_center[:,leg] = new_foot_center[:,leg]

        self.foot_center_updated = True
        self.update_foot_center()
        self.set_stride_length_bounds()

        self.server.set_succeeded()
        return

#==============================================================================
if __name__ == "__main__":
    try:
        gsc = gait_state_control_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
