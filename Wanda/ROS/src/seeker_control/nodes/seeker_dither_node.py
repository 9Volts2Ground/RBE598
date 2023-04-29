#!/usr/bin/env python3
#=====================
import copy
import numpy as np
import rospy
from sensor_msgs.msg import JointState

# Custom libraries
from seeker_control.init_seeker import init_seeker
from seeker_control.seeker_saturate import seeker_saturate
from topic_messages import topics

# Grab global parameters
NUM_SEEKER_JOINTS  = rospy.get_param( 'num_seeker_joints', default=2 )

# Initialize classes
top = topics.topics()

#==============================================================================
class seeker_dither_node():
    def __init__(self):
        rospy.init_node( "seeker_dither_node", anonymous=True )

        self.seeker_nominal = init_seeker()
        self.seeker_dithered = init_seeker()

        self.loop_hz = 30
        self.period = 1 / self.loop_hz
        self.rate = rospy.Rate( self.loop_hz )

        self.topic_in = rospy.get_param( f"{rospy.get_name()}/topic_in", default=top.seeker_nominal )
        self.topic_out = rospy.get_param( f"{rospy.get_name()}/topic_out", default=top.seeker_search )

        self.direction = [1, 1] # +/-1, az/el
        self.offset = [0.0, 0.0] # Current state of the dither angle

        # Set up dithering parameters
        # ToDo: make these launch parameters
        self.dither_speed = np.array( [ np.deg2rad(7), np.deg2rad( 12 ) ] ) * self.period
        self.dither_bound = np.array( [ np.deg2rad( 20 ), np.deg2rad( 6 ) ] )

        # Set up pubs/subs
        self.sub = rospy.Subscriber( self.topic_in, JointState, self.get_seeker_angles  )
        self.pub = rospy.Publisher( self.topic_out, JointState, queue_size=1 )

        # Run the controller
        self.dither_seeker()

    #======================================================
    def get_seeker_angles( self, seeker_angles ):
        self.seeker_nominal = copy.deepcopy( seeker_angles )

    #======================================================
    def dither_seeker( self ):

        while not rospy.is_shutdown():

            # Adjust the output states
            nominal_local = np.array( copy.deepcopy( self.seeker_nominal.position ) )
            switch_directions = [False, False]
            new_joints = [0.0, 0.0]

            for joint in range( NUM_SEEKER_JOINTS ):

                # Dither the angle
                self.offset[joint] += ( self.dither_speed[joint] * self.direction[joint] )

                if abs( self.offset[joint] ) >= self.dither_bound[joint]:
                    # We have exceeded how far we want to dither. Start moving the other direction
                    self.offset[joint] = self.dither_bound[joint] * self.direction[joint]
                    switch_directions[joint] = True

                new_joints[joint] = nominal_local[joint] + self.offset[joint]

            # Make sure we aren't exceeding the hardware bounds
            saturated_joints, saturated = seeker_saturate( new_joints )

            for joint in range( NUM_SEEKER_JOINTS ):

                # Check to see if we have exceeded seeker bounds.
                # If so, we need to start moving the other direction next time
                if saturated[joint]:
                    switch_directions[joint] = True
                    self.offset[joint] = saturated_joints[joint] - nominal_local[joint]

                # Update the new desired position
                self.seeker_dithered.position[joint] = nominal_local[joint] + self.offset[joint]

                # ToDo: velocity calculation

                # Flip the direction for next time around the loop
                if switch_directions[joint]:
                    self.direction[joint] *= -1

            # Send them out
            self.seeker_dithered.header.stamp = rospy.Time.now()
            self.pub.publish( self.seeker_dithered )

            self.rate.sleep()


#==============================================================================
if __name__ == "__main__":
    try:
        sd = seeker_dither_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
