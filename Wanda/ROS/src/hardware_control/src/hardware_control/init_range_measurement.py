#!/usr/bin/env python3
#=====================
import numpy as np
import rospy
from sensor_msgs.msg import Range

from system_globals import frames
frame = frames.frames()

#================================================
# Get global parameters
MAX_RANGE = rospy.get_param( 'ultrasonic/max_range', default=4.0 )

#==========================================================
def init_range_measurement():
    default_rng = Range()
    default_rng.header.frame_id = frame.ultrasonic
    default_rng.radiation_type = Range.ULTRASOUND
    default_rng.min_range = 0.02 # meters
    default_rng.max_range = MAX_RANGE # meters
    default_rng.field_of_view = 30 * np.pi/180
    default_rng.range = -np.inf
    return default_rng
