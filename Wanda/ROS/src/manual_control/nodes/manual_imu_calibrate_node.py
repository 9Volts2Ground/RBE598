#!/usr/bin/env python3
#=====================
import copy

# Standard ROS libraries
import actionlib
import rospy
from sensor_msgs.msg import Joy

# Custom libraries
from hardware_control.msg import ImuCalibrateAction, ImuCalibrateGoal
from manual_control.game_controller import game_controller

#==============================================================================
class manual_imu_calibrate_node():
    def __init__(self):
        rospy.init_node( "manual_imu_calibrate_node", anonymous=True )

        # Initialize controller instance for parsing
        self.controller = game_controller()
        self.controller_previous = copy.deepcopy( self.controller )

        # Set up the IMU calibration client
        self.client = actionlib.SimpleActionClient('imu_calibration_server', ImuCalibrateAction)

        # Define our calibration action goal
        self.goal = ImuCalibrateGoal()
        self.goal.calibrate_imu = True
        self.goal.calibration_duration = rospy.get_param( f"{rospy.get_name()}/calibration_duration", default=0.5 )

        self.client.wait_for_server() # Don't resume until the server gets turned on

        # Initialize subscription to joy controller
        rospy.Subscriber( "joy", Joy, self.imu_calibrate_client )

    #======================================================
    def imu_calibrate_client( self, joy ):

        # Parse Joy inputs
        self.controller_previous = copy.deepcopy( self.controller )
        self.controller.map_joy_to_controller( joy )

        if self.controller.x == 1 and self.controller_previous.x == 0:
            # The button is pressed. Send an action
            self.client.send_goal( self.goal )
            rospy.loginfo( "Starting IMU calibration action")

#==============================================================================
if __name__ == "__main__":
    try:
        imu_cal = manual_imu_calibrate_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
