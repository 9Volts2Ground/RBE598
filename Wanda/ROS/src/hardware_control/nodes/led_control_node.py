#!/usr/bin/env python3
import rospy

# Custom libraries
from hardware_control import Led
from topic_messages import topics
from walking_gait.msg import FootStates

#---------------------------------------------------
# Grab global parameters
NUM_LEGS = rospy.get_param( 'num_legs', default=6 )
LEG2LED = rospy.get_param( 'leg2led', default=[6, 0, 5, 1, 4, 2] )

#==============================================================================
class colors():
    def __init__(self):
        self.red =      [10, 0, 0]
        self.green =    [0, 0, 10]
        self.blue =     [0, 10, 0]

#----------------------------------------
# Set global flags and class instances
top = topics.topics()
led = Led.Led()
color = colors()

#==============================================================================
def led_control_node():
    rospy.init_node( "led_control_node", anonymous=True )

    for leg in range( NUM_LEGS ):
        rospy.Subscriber( top.leg_foot_states[leg], FootStates, set_leg_led, leg )

    rospy.spin()

#==============================================================================
def set_leg_led( foot_states, leg ):

    if foot_states.position > 0.0:
        led_color = color.blue
    else:
        led_color = color.green

    try:
        led.setColor( LEG2LED[leg], led_color )
    except:
        pass
        # print( f"led_color {leg_states.leg_num} (not lit) = {led_color}")

#==============================================================================
if __name__ == '__main__':
    try:
        led_control_node()
    except rospy.ROSInterruptException:
        pass
