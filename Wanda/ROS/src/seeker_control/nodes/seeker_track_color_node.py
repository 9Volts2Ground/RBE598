#!/usr/bin/env python3
#=====================
import rospy
from sensor_msgs.msg import JointState

from seeker_control.init_seeker import init_seeker
from seeker_control.seeker_saturate import seeker_saturate
from topic_messages import topics
from track_color.msg import target_track

# Initialize classes
top = topics.topics()

#==============================================================================
class seeker_track_color_node():
    def __init__(self):
        rospy.init_node( "seeker_track_color_node", anonymous = True )

        # Define class variables
        self.skr_state = init_seeker()

        # Gain values to tune
        self.target_centered_tolerance = 0.1
        self.seeker_gain = 0.03

        # Turn on publisher and subscribers
        self.pub = rospy.Publisher( top.seeker_nominal, JointState, queue_size = 1 )
        self.send_states() # Send the initial states out

        rospy.Subscriber( top.target_track_mode, target_track, self.seeker_tracking )

    #================================================================
    def seeker_tracking( self, target ):

        if target.tracking_state == target_track.TRACK:

            # We are tracking a target. See if we need to point the seeker az servo
            if abs( target.tgt_pos_in_frame.x ) > self.target_centered_tolerance:
                # The target is not centered in the frame. Point the seeker
                self.skr_state.position[0] += target.tgt_pos_in_frame.x * self.seeker_gain

            # We are tracking a target. See if we need to point the seeker el servo
            if abs( target.tgt_pos_in_frame.y ) > self.target_centered_tolerance:
                # The target is not centered in the frame. Point the seeker
                self.skr_state.position[1] += target.tgt_pos_in_frame.y * self.seeker_gain

        # Make sure the seeker angles don't get too big
        self.skr_state.position, saturated = seeker_saturate( self.skr_state.position )

        self.send_states()

    #================================================================
    def send_states( self ):
        self.skr_state.header.stamp = rospy.Time.now()
        self.pub.publish( self.skr_state )

#==============================================================================
if __name__ == "__main__":
    try:
        sc = seeker_track_color_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass