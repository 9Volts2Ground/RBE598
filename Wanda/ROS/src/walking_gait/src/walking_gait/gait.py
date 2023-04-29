#!/usr/bin/env python3
#=====================
import copy
import numpy as np

from walking_gait.msg import GaitStates

#====================================================================
class gait:
    #=======================================
    # Make this class a singleton
    __instance = None
    def __new__(cls, *args, **kwargs):
        if not gait.__instance:
            gait.__instance = object.__new__(cls)
        return gait.__instance

    #=======================================
    def __init__(self,
                 num_legs = 6,
                 gait_type = GaitStates.WAVE):

        #----------------------------------------
        # Process arguments
        #----------------------------------------
        self.num_legs = num_legs
        self.gait_type = gait_type

        #----------------------------------------
        # Kinematic parameters
        self.min_foot_height = 0.04
        self.foot_height = 0.04 # Z position of foot at peak stride in ground frame, meters

        # Foot z height polynomial trajectory coefficients
        self.z_traj_coeff = [0.0, 0.0, 0.0, 64.0, -192.0, 192.0, -64.0]

        #----------------------------------------
        # Properties to define how the gait works
        self.phase = 0.0 # Initialize to 0, increment as it walks
        self.beta = 0.0 # Ratio of how long each foot is on the ground

        # Which phase in the stride period each foot lifts off the ground
        self.phase_offset = np.zeros( self.num_legs )

        # Set up gait parameters based on gait type
        self.gait_specific_settings()

    #=======================================
    def update_gait_type( self, gait_type ):
        self.gait_type = gait_type
        self.gait_specific_settings()

    #=======================================
    def gait_specific_settings( self ):

        if self.num_legs == 4:
            # Can't do tripod or wave gait with only 4 legs
            self.gait_type == GaitStates.RIPPLE

        #-----------------------------------------
        # Process different gait types
        if self.gait_type == GaitStates.TRIPOD:
            self.beta = 1/2

            # Which phase in the stride period each foot lifts off the ground
            self.phase_offset = np.array( [0.0, 0.5,
                                           0.5, 0.0,
                                           0.0, 0.5] )

        elif self.gait_type == GaitStates.WAVE:
            self.beta = 4.0/6.0

            # Which phase in the stride period each foot lifts off the ground
            self.phase_offset = np.array( [4.0/6.0, 1.0/6.0,
                                           2.0/6.0, 5.0/6.0,
                                               0.0, 3.0/6.0] )

        else:
            # Force default to be the generic ripple gait
            self.gait_type == GaitStates.RIPPLE

            # Ratio of how long each foot is on the ground
            self.beta = (self.num_legs - 1) / self.num_legs

            # Which phase in the stride period each foot lifts off the ground
            self.phase_offset = np.zeros( self.num_legs )
            for leg in range( self.num_legs ):
                self.phase_offset[leg] = leg / self.num_legs

    #=======================================
    def increment_phase( self, max_foot_traveled, stride_length ):
        self.phase += max_foot_traveled / stride_length
        while ( self.phase > 1 ):
            # Make sure we roll over properly, decrement the phase until it is < 1
            self.phase -= 1

    #=======================================
    def update_foot_height( self, pose ):
        self.foot_height = max( self.min_foot_height,
                                pose.transform.translation.z )

#==============================================================================
if __name__ == "__main__":
    gt = gait()

    print("done")
