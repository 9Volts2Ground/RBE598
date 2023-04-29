#!/usr/bin/env python3
#=====================
from geometry_msgs.msg import TransformStamped
import rospy
from sensor_msgs.msg import JointState
import tf2_ros
from tf.transformations import quaternion_from_matrix

# Custom libraries and class instances
from system_globals.broadcast_static_frame import broadcast_static_frame
from system_globals import frames
from system_globals import transformation_frames
from topic_messages import topics

NUM_LEGS = rospy.get_param( 'num_legs', default=6 )

# Initialize classes
frame = frames.frames()
top = topics.topics()
tf = transformation_frames.transformation_frames()

#==============================================================================
class legs_fkine_frames_node():
    def __init__(self):
        rospy.init_node( "legs_fkine_frames_node", anonymous=True )

        self.tf2_broadcaster =  [ tf2_ros.TransformBroadcaster() for leg in range( NUM_LEGS ) ]
        for leg in range( NUM_LEGS ):
            rospy.Subscriber( top.leg_joint_states[leg], JointState, self.calc_leg_fkine, leg )

    #==============================================================================
    def calc_leg_fkine( self, joint_state, leg ):
        # Broadcast body2imu
        broadcast_static_frame( tf.T_body2imu,
                                frame.body,
                                frame.imu )

        # Calculate transforms to each joint
        T_shoulder2coxa = tf.transform_shoulder2coxa( joint_state.position[0] )
        T_coxa2femur = tf.transform_coxa2femur( joint_state.position[1] )
        T_femur2tibia = tf.transform_femur2tibia( joint_state.position[2] )
        T_tibia2foot = tf.transform_tibia2foot()

        # Broadcast body2shoulder
        broadcast_static_frame( tf.T_body2shoulder[leg],
                                frame.body,
                                frame.shoulder[leg] )

        # Broadcast shoulder2coxa
        self.broadcast_fkine( leg,
                              T_shoulder2coxa,
                              frame.shoulder[ leg ],
                              frame.coxa[ leg ],
                              joint_state.header.stamp )

        # Broadcast coxa2femur
        self.broadcast_fkine( leg,
                              T_coxa2femur,
                              frame.coxa[ leg ],
                              frame.femur[ leg ],
                              joint_state.header.stamp )

        # Broadcast femur2tibia
        self.broadcast_fkine( leg,
                              T_femur2tibia,
                              frame.femur[ leg ],
                              frame.tibia[ leg ],
                              joint_state.header.stamp )

        # Broadcast tibia2foot
        self.broadcast_fkine( leg,
                              T_tibia2foot,
                              frame.tibia[ leg ],
                              frame.foot[ leg ],
                              joint_state.header.stamp )

    #==============================================================================
    def broadcast_fkine( self, leg_num, T_x2y, parent, child, time ):
        trans = TransformStamped()
        trans.header.stamp = time

        # Package and send transform
        trans.header.frame_id = parent
        trans.child_frame_id = child
        trans.transform.translation.x = T_x2y[0,3]
        trans.transform.translation.y = T_x2y[1,3]
        trans.transform.translation.z = T_x2y[2,3]

        q = quaternion_from_matrix( T_x2y )
        trans.transform.rotation.x = q[0]
        trans.transform.rotation.y = q[1]
        trans.transform.rotation.z = q[2]
        trans.transform.rotation.w = q[3]

        self.tf2_broadcaster[leg_num].sendTransform( trans )

#==============================================================================
if __name__ == "__main__":
    try:
        fkine = legs_fkine_frames_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
