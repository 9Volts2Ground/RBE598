#!/usr/bin/env python3
#=====================
import copy
import numpy as np
import rospy

# Standard ROS libraries
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy

# Custom libraries
from manual_control.game_controller import game_controller
from manual_control.stick_mode import stick_mode
from manual_control.update_joystick_mode import update_joystick_mode
from system_globals import frames
from topic_messages import topics

frame = frames.frames()
top = topics.topics()

#==============================================================================
class manual_walk_twist_control_node():
    def __init__(self):
        rospy.init_node( "manual_walk_twist_control_node", anonymous=True )

        # Initialize controller instance for parsing
        self.controller = game_controller()
        self.controller_previous = copy.deepcopy( self.controller )

        # Initialize gait twist state
        self.tw = TwistStamped()
        self.init_twist()

        # Define gait parameters/gains
        self.max_velocity = 0.06 # m/s
        self.max_angular_velocity = np.pi/10 # rad/s

        # Use the bumpers to switch each joystick control mode: walk vs pose
        self.l_stick_mode = stick_mode.WALK
        self.r_stick_mode = stick_mode.WALK

        # Define publishers
        self.pub_twist = rospy.Publisher( top.walking_twist_commanded, TwistStamped, queue_size = 1 )

        # Initialize subscription to joy controller
        rospy.Subscriber( "joy", Joy, self.manual_walk_twist_control )

    #======================================================
    def manual_walk_twist_control( self, joy ):

        # Parse Joy inputs
        self.controller_previous = copy.deepcopy( self.controller )
        self.controller.map_joy_to_controller( joy )

        # Check if we are controling walking gait or body pose
        self.r_stick_mode, self.l_stick_mode = update_joystick_mode( self.controller,
                                                                     self.controller_previous,
                                                                     self.r_stick_mode,
                                                                     self.l_stick_mode )

        if self.l_stick_mode == stick_mode.WALK:
            # Calculate linear vector from left joystick
            scaled_linear_vec = np.array( [ self.controller.l_stick.fwd_back,
                                            -self.controller.l_stick.right_left,
                                            0.0 ] ) * self.max_velocity # No vertical velocity vector. We can't fly yet :/
            self.tw.twist.linear.x = scaled_linear_vec[0]
            self.tw.twist.linear.y = scaled_linear_vec[1]
        else:
            self.clear_twist_linear() # Make sure we stop walking if we switch modes

        if self.r_stick_mode == stick_mode.WALK:
            # Calculate angular vector from right joystick
            # Note: positive Z is counter-clockwise, hence - right_left
            self.tw.twist.angular.z = -self.controller.r_stick.right_left * self.max_angular_velocity
        else:
            self.clear_twist_angular()

        # Send out desired walking gait twist vector
        self.tw.header.stamp = rospy.Time.now()
        self.pub_twist.publish( self.tw )

    #======================================================
    def init_twist( self ):
        ''' Initializes commanded twist vector '''
        self.tw.header.stamp = rospy.Time.now()
        self.tw.header.frame_id = frame.ground

        self.clear_twist_linear()
        self.clear_twist_angular()

    #======================================================
    def clear_twist_linear( self ):
        ''' Zero out the linear component of the twist vector '''
        self.tw.twist.linear.x = 0.0
        self.tw.twist.linear.y = 0.0
        self.tw.twist.linear.z = 0.0

    #======================================================
    def clear_twist_angular( self ):
        ''' Zero out the angular component of the twist vector '''
        self.tw.twist.angular.x = 0.0
        self.tw.twist.angular.y = 0.0
        self.tw.twist.angular.z = 0.0

#==============================================================================
if __name__ == "__main__":
    try:
        wc = manual_walk_twist_control_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
