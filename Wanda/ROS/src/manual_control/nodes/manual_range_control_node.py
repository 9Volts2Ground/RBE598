#!/usr/bin/env python3
#=====================
import actionlib
import copy
import rospy

# Standard ROS libraries
from sensor_msgs.msg import Joy

# Custom libraries
from manual_control.game_controller import game_controller
from range_sensor_control.msg import SensorStateAction, SensorStateGoal
from system_globals import frames
from topic_messages import topics

frame = frames.frames()
top = topics.topics()

#==============================================================================
class manual_range_control_node():
    def __init__(self):
        rospy.init_node( "manual_range_control_node", anonymous=True )

        # Initialize controller instance for parsing
        self.controller = game_controller()
        self.controller_previous = copy.deepcopy( self.controller )

        # Initialize the ultrasonic range sensor state to inactive
        self.range_state = SensorStateGoal()
        self.range_state.state = SensorStateGoal.INACTIVE

        self.client = actionlib.SimpleActionClient( 'range_sensor_state_server',
                                                    SensorStateAction )
        
        # Don't resume until the server gets turned on
        self.client.wait_for_server()

        # Initialize subscription to joy controller
        rospy.Subscriber( "joy", Joy, self.manual_range_control )

    #======================================================
    def manual_range_control( self, joy ):

        # Parse Joy inputs
        self.controller_previous = copy.deepcopy( self.controller )
        self.controller.map_joy_to_controller( joy )

        if self.controller.b == 1 and self.controller_previous.b == 0:
            # The button is pressed. Toggle state
            if self.range_state.state == SensorStateGoal.ACTIVE:
                self.range_state.state = SensorStateGoal.INACTIVE
            else:
                self.range_state.state = SensorStateGoal.ACTIVE

            # Send an action to the server
            self.client.send_goal( self.range_state )

#==============================================================================
if __name__ == "__main__":
    try:
        rc = manual_range_control_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
