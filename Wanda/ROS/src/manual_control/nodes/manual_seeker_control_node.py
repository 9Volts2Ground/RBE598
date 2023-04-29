#!/usr/bin/env python3
#=====================
import copy
import rospy

# Standard ROS libraries
from sensor_msgs.msg import JointState, Joy

# Custom libraries
from manual_control.game_controller import game_controller
from manual_control.stick_mode import stick_mode
from manual_control.update_joystick_mode import update_joystick_mode
from seeker_control.seeker_saturate import seeker_saturate
from system_globals import frames
from topic_messages import topics

frame = frames.frames()
top = topics.topics()

#==============================================================================
class manual_seeker_control_node():
    def __init__(self):
        rospy.init_node( "manual_seeker_control_node", anonymous=True )

        # Initialize controller instance for parsing
        self.controller = game_controller()
        self.controller_previous = copy.deepcopy( self.controller )

        # Use the bumpers to switch each joystick control mode: walk vs pose
        self.r_stick_mode = stick_mode.WALK
        self.l_stick_mode = stick_mode.WALK

        # Initialize seeker states
        self.skr_state = JointState() #  Initialize seeker state topic
        self.skr_state.header.frame_id = frame.neck_static
        self.skr_state.name = ["azimuth", "elevation"]
        self.skr_state.position = [ 0.0, 0.0 ]
        self.skr_state.velocity = [ 0.0, 0.0 ]
        self.skr_state.effort = [ 0.0, 0.0 ]

        # Define publisher
        self.pub_skr = rospy.Publisher( top.seeker_nominal, JointState, queue_size = 1 )

        # If we are running with the simulation, wait for the clock to turn on
        while rospy.get_time() == 0 and not rospy.is_shutdown():
            rospy.loginfo_once( "manual_seeker_control_node.py waiting for /clock time to be published")

        # Initialize subscription to joy controller
        rospy.Subscriber( "joy", Joy, self.manual_seeker_control )

    #======================================================
    def manual_seeker_control( self, joy ):

        # Parse Joy inputs
        self.controller_previous = copy.deepcopy( self.controller )
        self.controller.map_joy_to_controller( joy )

        self.r_stick_mode, self.l_stick_mode = update_joystick_mode( self.controller,
                                                                     self.controller_previous,
                                                                     self.r_stick_mode,
                                                                     self.l_stick_mode )

        # Set the time stamp
        self.skr_state.header.stamp = rospy.Time.now()

        # Left/right controlled by the triggers
        self.skr_state.position[0] += 0.04 * ((self.controller.r_trigger-1)-(self.controller.l_trigger-1))/2

        # Up/down controlled by the right stick, but only in "walk" mode, since
        # the joystick gets used to control the body pose too
        if self.r_stick_mode == stick_mode.WALK:
            self.skr_state.position[1] += 0.02 * self.controller.r_stick.fwd_back

        # Keep our seeker angles from going out of bounds
        self.skr_state.position, saturated = seeker_saturate( self.skr_state.position )

        self.pub_skr.publish( self.skr_state )

#==============================================================================
if __name__ == "__main__":
    try:
        sc = manual_seeker_control_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
