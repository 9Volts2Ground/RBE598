#!/usr/bin/env python3
#=====================
import numpy as np
import rospy

# Custom classes
from hardware_control.init_range_measurement import init_range_measurement
from hardware_control.srv import ultrasonic_range, ultrasonic_rangeResponse
from hardware_control.Ultrasonic import Ultrasonic
from system_globals import frames

frame = frames.frames()

#==============================================================================
class ultrasonic_range_sensor_server():
    def __init__(self):
        rospy.init_node( 'ultrasonic_range_sensor_server', anonymous=False )

        # Initialize ultrasonic range sensor class
        self.sonic = Ultrasonic()

        # Initilaize range sensor topic
        self.response = ultrasonic_rangeResponse()
        self.response.range = init_range_measurement()
        self.response.range.header.stamp = rospy.Time.now()

        # If we are running with the simulation, wait for the clock to turn on
        while rospy.get_time() == 0 and not rospy.is_shutdown():
            rospy.loginfo_once( "ultrasonic_range_sensor_server.py waiting for /clock time to be published")

        # Kick off the service
        serve = rospy.Service( 'ultrasonic_range_sensor',
                                ultrasonic_range,
                                self.ping_range_sensor )
        rospy.loginfo("ultrasonic_range_sensor server ready...")
        rospy.spin()

    #==========================================================================
    def ping_range_sensor( self, ultrasonic_range ):
        ''' This is where we handle the range sensor hardware and take a measurement'''

        # Call the hardware, get a measurement
        measured_range, returned = self.sonic.get_distance()

        self.response.range.header.stamp = rospy.Time.now()
        if returned:
            self.response.range.range = measured_range
        else:
            self.response.range.range = -np.inf

        return self.response

#==============================================================================
if __name__ == "__main__":
    try:
        rng_sense_srv = ultrasonic_range_sensor_server()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
