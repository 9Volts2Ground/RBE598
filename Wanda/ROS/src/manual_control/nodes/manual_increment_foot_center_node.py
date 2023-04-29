#!/usr/bin/env python3
#=====================
import copy

# Standard ROS libraries
import actionlib
import rospy
from sensor_msgs.msg import Joy

# Custom libraries
from manual_control.game_controller import game_controller
from walking_gait.msg import IncrementFootCenterAction, IncrementFootCenterGoal

#==============================================================================
class manual_increment_foot_center_node():
    def __init__(self):
        rospy.init_node( "manual_increment_foot_center_node", anonymous=True )

        # Initialize controller instance for parsing
        self.controller = game_controller()

        # Set up the IMU calibration client
        self.client = actionlib.SimpleActionClient('foot_center_server',
                                                   IncrementFootCenterAction)

        # Define our calibration action goal
        self.goal = IncrementFootCenterGoal()
        self.increment_mag = rospy.get_param( f"{rospy.get_name()}/increment_mag", default=0.001 )

        self.client.wait_for_server() # Don't resume until the server gets turned on

        # Initialize subscription to joy controller
        rospy.Subscriber( "joy", Joy, self.increment_foot_center_client )

    #======================================================
    def increment_foot_center_client( self, joy ):

        # Parse Joy inputs
        self.controller.map_joy_to_controller( joy )

        increment = 0
        increment += self.controller.r_shoulder
        increment -= self.controller.l_shoulder

        if increment != 0:
            self.goal.increment_mag = increment * self.increment_mag
            self.client.send_goal( self.goal )

#==============================================================================
if __name__ == "__main__":
    try:
        foot_inc = manual_increment_foot_center_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
