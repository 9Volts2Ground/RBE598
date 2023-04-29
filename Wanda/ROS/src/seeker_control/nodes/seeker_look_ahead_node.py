#!/usr/bin/env python3
#=====================
import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState

from seeker_control.init_seeker import init_seeker
from seeker_control.seeker_saturate import seeker_saturate
from topic_messages import topics

# Initialize classes
top = topics.topics()

#==============================================================================
class seeker_look_ahead_node():
    def __init__(self):
        rospy.init_node( "seeker_look_ahead_node", anonymous = True )

        # Define class variables
        self.skr_state = init_seeker()

        # Gain values to tune
        self.twist_az_gain = 0.75

        self.chin_up = np.deg2rad( 25 )

        # Turn on publisher and subscribers
        self.pub = rospy.Publisher( top.seeker_nominal, JointState, queue_size = 1 )
        self.send_states() # Send the initial states out

        rospy.Subscriber( top.walking_twist_filtered, TwistStamped, self.look_ahead )

    #================================================================
    def look_ahead( self, twist ):

        # Point the az angle in the direction we are walking
        az = 0.0
        if abs( twist.twist.linear.x ) > 0.001:
            az = np.arctan2( twist.twist.linear.y, twist.twist.linear.x )

        # If we are spinning, point the seeker that direction a bit
        az += twist.twist.angular.z * self.twist_az_gain

        # Package the desired angles up
        self.skr_state.position[0] = az

        # Keep the "chin up" to prevent pointing seeker at ground clutter
        self.skr_state.position[1] = self.chin_up

        # Make sure the seeker angles don't get too big
        self.skr_state.position, saturated = seeker_saturate( self.skr_state.position )

        self.send_states()

    #================================================================
    def send_states( self ):
        now = rospy.Time.now()
        self.skr_state.header.stamp = now
        self.pub.publish( self.skr_state )

#==============================================================================
if __name__ == "__main__":
    try:
        sl = seeker_look_ahead_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass