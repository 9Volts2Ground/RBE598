#!/usr/bin/env python3
#=====================
import rospy
from sensor_msgs.msg import JointState

# Custom libraries
from seeker_control.broadcast_seeker_fkine import broadcast_seeker_fkine
from topic_messages import topics

# Initialize classes
top = topics.topics()

#==============================================================================
class seeker_fkine_frames_node():
    def __init__(self):
        rospy.init_node( "seeker_fkine_frames_node", anonymous=True )

        self.topic_in = rospy.get_param( f"{rospy.get_name()}/topic_in", default=top.seeker_filtered )

        rospy.Subscriber( self.topic_in, JointState, self.send_seeker_fkine )

    #==================================================================
    def send_seeker_fkine( self, seeker_state ):

        broadcast_seeker_fkine( seeker_state.position,
                                seeker_state.header.stamp )

#==============================================================================
if __name__ == "__main__":
    try:
        skr_fkine = seeker_fkine_frames_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
