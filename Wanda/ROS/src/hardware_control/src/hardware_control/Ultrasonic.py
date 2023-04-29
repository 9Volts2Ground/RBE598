#!/usr/bin/env python3
import copy
import numpy as np
import rospy

#================================================
# Get global parameters
MAX_RANGE = rospy.get_param( 'ultrasonic/max_range', default=4.0 )
IS_WANDA = rospy.get_param( 'IS_WANDA', default=False )

if IS_WANDA:
    import RPi.GPIO as GPIO

#==============================================================================
class Ultrasonic:
    def __init__(self):
        self.trigger_pin = rospy.get_param( 'ultrasonic/transmit_pin', default=27 ) # transmit pin on the RPi
        self.echo_pin = rospy.get_param( 'ultrasonic/receive_pin', default=22 ) # receive pin on the RPi
        self.half_sonic_speed = 344 / 2 # (Speed of sound in m/s) / (tx + rx)
        self.max_send_return_time = MAX_RANGE / self.half_sonic_speed
        self.calibration_offset = rospy.get_param( 'ultrasonic/calibration_offset', default=0.0 ) # meters
        self.pulse_time = 0.00015 # seconds

        # Default to both pins being off
        self.trigger_state = False
        self.echo_state = False

        if IS_WANDA:
            # Setup GPIO pins for the sensor only on hardware
            GPIO.setwarnings( False )
            GPIO.setmode( GPIO.BCM )
            GPIO.setup( self.trigger_pin, GPIO.OUT )
            GPIO.setup( self.echo_pin, GPIO.IN )

        self.set_transmit_state( False ) # Default to not pulsing signal

    #======================================================
    def set_transmit_state( self, state ):
        """ Sets pin state. Only sends command to pin if on hardware
        """
        self.trigger_state = state
        if IS_WANDA:
            # Only try to set the pin if running on hardware
            GPIO.output(self.trigger_pin, self.trigger_state)

    #======================================================
    def get_receive_state( self ):
        if IS_WANDA:
            self.echo_state = GPIO.input( self.echo_pin )
        return self.echo_state

    #======================================================
    def send_trigger_pulse( self ):
        """ Pulses the sonic module for 0.00015s to send out 40kHz signal """
        self.set_transmit_state( True ) # Send signal
        rospy.sleep( self.pulse_time )
        self.set_transmit_state( False ) # Stop sending signal

    #======================================================
    def echo_return_time( self, desired_state, timeout ):
        """
        Waits for state of pin to change to find time
        Args:
            desired_state (bool): Desired state we want the pin to obtain
            timeout ([type]): [description]
        Returns:
            [type]: [description]
        """
        start_time = rospy.get_time()
        current_time = copy.deepcopy( start_time )

        self.get_receive_state()

        while ( self.echo_state != desired_state ) and ( current_time - start_time < timeout ):
            self.get_receive_state()
            current_time = rospy.get_time()

        return current_time, self.echo_state

    #======================================================
    def get_distance(self):
        """ Pulses HC-SR04 sensor 3x and chooses the average distance """

        range = -np.inf
        returned = False

        self.send_trigger_pulse()

        # Wait for the sensor to start receiving signal back
        start_time, returned = self.echo_return_time( True, self.max_send_return_time )

        if returned:
            # If we got signal back, loop until we lose signal again
            finish_time, stopped = self.echo_return_time( False, self.max_send_return_time )

            # Calculate the measured range based on how long it took
            range = ( finish_time - start_time ) * self.half_sonic_speed + self.calibration_offset

        if range > MAX_RANGE:
            # Invaid measurement, clear it
            range = -np.inf
            returned = False

        return range, returned

    #======================================================
    def average_distances( self, num_pulses ):
        """
        Pulses the sensor num_pulses times, returns the average and std dev range
        Args:
            num_pulses (int): Number of times to send and receive pulses
        Returns:
            mean (float): Average distance recorded. If no range found, returns 0.0
            std_dev (float): Standard deviation of ranges measured. If no range found, returns 0.0
        """
        distances = []
        ever_returned = False

        # Pulse the specified number of times
        for pulse in range( num_pulses ):

            distance, returned = self.get_distance()

            # Only store if we think we found something
            if returned:
                ever_returned = True
                distances.append( distance )

        if ever_returned:
            return np.mean( distances ), np.std( distances )
        else:
            return -np.inf, -np.inf


#==============================================================================
if __name__ == '__main__':
    sonic = Ultrasonic()
    while True:
        range, stddev= sonic.average_distances( 5 )
        print("Range, std: ", range, stddev)

