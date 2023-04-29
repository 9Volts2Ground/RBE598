#!/usr/bin/env python3
#=====================
import copy
import rospy

# Standard ROS message type
from geometry_msgs.msg import TwistStamped

# Custom libraries
from topic_messages import topics

top = topics.topics()

#==============================================================================
class twist_lowpass_filter_node():
    def __init__(self):
        rospy.init_node( "twist_lowpass_filter_node", anonymous=True )

        self.twist_old = TwistStamped()
        self.twist_old.twist = self.init_twist( self.twist_old.twist )

        self.twist_new = TwistStamped()
        self.twist_new.twist = self.init_twist( self.twist_new.twist )

        self.period = 1.0/5.0 # Filter frequency

        self.pub = rospy.Publisher( top.walking_twist_filtered, TwistStamped, queue_size=1 )

        rospy.Subscriber( top.walking_twist_commanded, TwistStamped, self.filter_twist )

    #======================================================
    def filter_twist( self, twist_in ):

        self.twist_new = copy.deepcopy( twist_in )

        # Filter the components of the twist
        self.twist_new.twist.linear.x = self.low_pass_filter(
                                            self.twist_new.twist.linear.x,
                                            self.twist_old.twist.linear.x )
        self.twist_new.twist.linear.y = self.low_pass_filter(
                                            self.twist_new.twist.linear.y,
                                            self.twist_old.twist.linear.y )
        self.twist_new.twist.linear.z = self.low_pass_filter(
                                            self.twist_new.twist.linear.z,
                                            self.twist_old.twist.linear.z )

        self.twist_new.twist.angular.x = self.low_pass_filter(
                                            self.twist_new.twist.angular.x,
                                            self.twist_old.twist.angular.x )
        self.twist_new.twist.angular.y = self.low_pass_filter(
                                            self.twist_new.twist.angular.y,
                                            self.twist_old.twist.angular.y )
        self.twist_new.twist.angular.z = self.low_pass_filter(
                                            self.twist_new.twist.angular.z,
                                            self.twist_old.twist.angular.z )

        # Send out the filtered twist
        self.pub.publish( self.twist_new )

        # Make sure to save this new command for next time
        self.twist_old = copy.deepcopy( self.twist_new )

    #======================================================
    def low_pass_filter( self, state_current, state_old, period=None ):
        if period == None:
            # Use the default filter frequency if none is specified
            period = self.period

        state_new = period * state_old + (1-period) * state_current

        return state_new

    #======================================================
    def init_twist( self, twist ):
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        return twist

#==============================================================================
if __name__ == "__main__":
    try:
        tf = twist_lowpass_filter_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

