#!/usr/bin/env python3
#=====================
import copy
import numpy as np
import rospy
from sensor_msgs.msg import JointState

# Custom libraries
from hardware_control import Servo
from topic_messages import topics

#---------------------------------------------------
# Grab global parameters
NUM_JOINTS = rospy.get_param( 'num_joints', default=3 )
NUM_LEGS = rospy.get_param( 'num_legs', default=6 )
# Leg motor params
LEG_CHANNEL = np.reshape( rospy.get_param('leg_servos/channel'), (3,6) )
MOTOR_MAX = np.reshape( rospy.get_param('leg_servos/max'), (3,6) )
MOTOR_MIN = np.reshape( rospy.get_param('leg_servos/min'), (3,6) )
MOTOR_ORIENTATION = np.reshape( rospy.get_param( 'leg_servos/orientation'), (3,6) )
MOTOR_CENTER = np.reshape( rospy.get_param( 'leg_servos/center' ), (3,6) )
# Seeker motor params
NUM_SEEKER_JOINTS = rospy.get_param( 'num_seeker_joints', default=2 )
SEEKER_CHANNEL = rospy.get_param('seeker_servos/channel')
SEEKER_MAX = rospy.get_param('seeker_servos/max')
SEEKER_MIN = rospy.get_param('seeker_servos/min')
SEEKER_ORIENTATION = rospy.get_param( 'seeker_servos/orientation')
SEEKER_CENTER = rospy.get_param( 'seeker_servos/center' )

#----------------------------------------
# Set global flags and class instances
top = topics.topics()
servo = Servo.Servo()

rad2deg = 180/np.pi

#==============================================================================
class move_motors_node():
    def __init__(self):

        rospy.init_node( "move_motors_node", anonymous=True )

        # Note: the stock servos work on 50Hz PWM,
        # so they can't accept signal much faster than 10Hz
        self.rate = rospy.Rate( 10 )

        # Kick off unique subscriber and states for each leg
        self.leg_states = []
        self.leg_sub = []
        for leg in range( NUM_LEGS ):
            self.leg_states.append( JointState() )
            self.leg_states[leg].position = [0.0, np.deg2rad(80.0), np.deg2rad(120.0)] # Init to nothing until we get real angles
            self.leg_sub.append( rospy.Subscriber( top.leg_joint_states[leg],
                                                   JointState,
                                                   self.grab_leg_states,
                                                   leg ) )

        # Also need subscriber and states for seeker
        self.seeker_states = JointState()
        self.seeker_states.position = [0.0, 0.0] # Init to nothing until we get real angles
        self.seeker_sub = rospy.Subscriber( top.seeker_filtered,
                                            JointState,
                                            self.grab_seeker_states )

        # Kick off the while-loop that sends motor PWM signal
        self.move_motors()

    #==============================================================================
    def move_motors( self ):

        while not rospy.is_shutdown():

            #--------------------------------------------------------
            # Move all the legs
            for leg in range( NUM_LEGS ):
                for joint in range( NUM_JOINTS ):

                    # Convert angle to degrees, offset it by the hardware 0 value
                    # Offset calculation based on center of calibrated servo
                    angle_command = self.leg_states[leg].position[joint] \
                        * rad2deg * MOTOR_ORIENTATION[joint,leg] \
                        + MOTOR_CENTER[joint,leg]

                    # Saturate the motor commands to min/max values
                    # Note: this runs more efficiently than np.max( np.min() )
                    if angle_command > MOTOR_MAX[joint,leg]:
                        angle_command = MOTOR_MAX[joint,leg]
                    elif angle_command < MOTOR_MIN[joint,leg]:
                        angle_command = MOTOR_MIN[joint,leg]

                    # Ensure angle is an int to pass into setServoAngle()
                    angle_command = int( angle_command )

                    try:
                        servo.setServoAngle( LEG_CHANNEL[joint, leg], angle_command )
                    except:
                        rospy.logerr(f"Could not move servo {joint} on leg {leg}")

            #--------------------------------------------------------
            # Also move the seeker. Same process, but different data sources
            for joint in range( NUM_SEEKER_JOINTS ):

                # Convert angle to degrees, offset it by the hardware 0 value
                # Offset calculation based on center of calibrated servo
                angle_command = self.seeker_states.position[joint] \
                    * rad2deg * SEEKER_ORIENTATION[joint] \
                    + SEEKER_CENTER[joint]

                # Saturate the motor commands to min/max values
                if angle_command > SEEKER_MAX[joint]:
                    angle_command = SEEKER_MAX[joint]
                elif angle_command < SEEKER_MIN[joint]:
                    angle_command = SEEKER_MIN[joint]

                # Ensure angle is an int to pass into setServoAngle()
                angle_command = int( angle_command )

                try:
                    servo.setServoAngle( SEEKER_CHANNEL[joint], angle_command )
                except:
                    rospy.logerr(f"Could not move {self.seeker_states.name[joint]} seeker servo")


            # Wait to loop again
            self.rate.sleep()

    #==============================================================================
    def grab_leg_states( self, leg_state, leg ):
        ''' Stash a copy of the leg states we got from the subscriber'''
        self.leg_states[leg] = copy.deepcopy( leg_state )

    #==============================================================================
    def grab_seeker_states( self, seeker_state ):
        ''' Stash a copy of the seeker states we got from the subscriber'''
        self.seeker_states = copy.deepcopy( seeker_state )

#==============================================================================
if __name__ == '__main__':
    try:
        move = move_motors_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
