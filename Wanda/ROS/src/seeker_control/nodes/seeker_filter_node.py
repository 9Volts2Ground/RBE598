#!/usr/bin/env python3
#=====================
import copy
import rospy
from sensor_msgs.msg import JointState

# Custom libraries
from seeker_control.init_seeker import init_seeker
from seeker_control.seeker_saturate import seeker_saturate
from topic_messages import topics

# Initialize classes
top = topics.topics()

#==============================================================================
class seeker_filter_node():
    def __init__(self):
        rospy.init_node( "seeker_filter_node", anonymous=True )

        # Define class variables
        self.seeker_state_desired = init_seeker()
        self.seeker_state_filtered = init_seeker()

        self.loop_hz = 30
        self.period = 1 / self.loop_hz
        self.rate = rospy.Rate( self.loop_hz )

        self.filter_gain = 0.4

        self.topic_in = rospy.get_param( f"{rospy.get_name()}/topic_in", default=top.seeker_nominal )
        self.topic_out = rospy.get_param( f"{rospy.get_name()}/topic_out", default=top.seeker_filtered )

        # Set up pubs/subs
        self.sub = rospy.Subscriber( self.topic_in, JointState, self.get_seeker_angles  )
        self.pub = rospy.Publisher( self.topic_out, JointState, queue_size=1 )

        # Run the filter
        self.filter_seeker()

    #======================================================
    def get_seeker_angles( self, seeker_angles ):
        self.seeker_state_desired = copy.deepcopy( seeker_angles )

    #======================================================
    def filter_seeker( self ):

        while not rospy.is_shutdown():

            az_new = self.low_pass_filter(
                self.seeker_state_filtered.position[0],
                self.seeker_state_desired.position[0],
                self.filter_gain
            )

            el_new = self.low_pass_filter(
                self.seeker_state_filtered.position[1],
                self.seeker_state_desired.position[1],
                self.filter_gain
            )

            # Make sure we don't exceed hardware limits
            [az_new, el_new], saturated = seeker_saturate( [az_new, el_new] )

            # Calculate (roughly) seeker angle velocity
            az_dot = ( az_new - self.seeker_state_filtered.position[0] ) * self.period
            el_dot = ( el_new - self.seeker_state_filtered.position[1] ) * self.period

            # Save off states
            self.seeker_state_filtered.position[0] = az_new
            self.seeker_state_filtered.position[1] = el_new
            self.seeker_state_filtered.velocity[0] = az_dot
            self.seeker_state_filtered.velocity[1] = el_dot

            # Publish them out
            self.seeker_state_filtered.header.stamp = rospy.Time.now()
            self.pub.publish( self.seeker_state_filtered )

            self.rate.sleep()

    #======================================================
    def low_pass_filter( self, state_current, state_desired, period=None ):
        if period == None:
            # Use the default filter frequency if none is specified
            period = self.period

        state_new = period * state_desired + (1-period) * state_current

        return state_new

#==============================================================================
if __name__ == "__main__":
    try:
        sf = seeker_filter_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
