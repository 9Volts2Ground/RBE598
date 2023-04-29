#!/usr/bin/env python3
#=====================
import copy
import rospy
import numpy as np

# Standard ROS message type
import actionlib
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Range

from range_sensor_control.msg import SensorStateAction, SensorStateGoal
from seeker_control.init_seeker import init_seeker
from system_globals import frames
from topic_messages import topics
from track_color.msg import target_states
from track_color.msg import target_track

frame = frames.frames()
top = topics.topics()

#==============================================================================
class walk_control_node():
    def __init__(self):
        rospy.init_node( "walk_control_node", anonymous = True )

        # Initialize topic data to work with
        self.target_states = target_states()

        self.range_data = Range()
        self.range_data.range = -np.inf # Init to no return

        # Set the initial state of the ultrasonic range sensor
        self.range_state = SensorStateGoal()
        self.range_state.state = SensorStateGoal.INACTIVE
        
        self.client = actionlib.SimpleActionClient( 'range_sensor_state_server',
                                                    SensorStateAction )
        
        # Don't resume until the server gets turned on
        self.client.wait_for_server()

        self.seeker_states = init_seeker()

        #----------------------------------
        # Define local variables. Detection states
        self.target_search_mode = target_track.SEARCH
        self.previous_target_search_mode = self.target_search_mode
        self.azimuth_position = 0.0
        self.elevation_position = 0.0
        self.target_lost_timeout = 2.0
        self.target_acquire_distance = 0.5 # meters

        # Other logic variables
        self.angular_moment = 0.15 # Approximate distance from body to foot
        self.max_velocity = 0.03 # m/s
        self.target_centered_tolerance = 0.1 # Acceptable distance from center of image, % of frame
        self.seeker_turned_tolerance = 0.436 # ~25 degrees off center threshold to add spin to the robot
        self.seeker_az_gain = 0.1
        self.target_az_side = 1 # >0 = left, <0 = right
        self.spin_gain = 0.05 # Scales error relative to image position into angular velocity

        # If we are running with the simulation, wait for the clock to turn on
        while rospy.get_time() == 0 and not rospy.is_shutdown():
            rospy.loginfo_once( "walk_control_node.py waiting for /clock time to be published")
        self.target_lost_time = rospy.get_time()

        # Turn on publisher and subscribers
        self.pub = rospy.Publisher( top.walking_twist_commanded, TwistStamped, queue_size = 1 )
        self.pub_track = rospy.Publisher( top.target_track_mode, target_track, queue_size = 1 )
        rospy.Subscriber( top.range_data_raw, Range, self.grab_range_states )
        rospy.Subscriber( top.seeker_filtered, JointState, self.grab_seeker_states )
        rospy.Subscriber( top.target_image_track, target_states, self.grab_target_states )

        self.walk_control_node()

    #======================================================
    def walk_control_node( self ):
        rate = rospy.Rate( 10 )

        rospy.loginfo("Entering walking while loop...")

        while not rospy.is_shutdown():

            # Grab static, local copies of states to work with
            target_state = copy.deepcopy( self.target_states )
            range_data = copy.deepcopy( self.range_data )

            #------------------------------------------------------------------------
            # Determine which state we need to be in
            if self.previous_target_search_mode == target_track.ACQUIRED:
                # Stay acquired
                self.target_search_mode = target_track.ACQUIRED
            elif target_state.target_found:
                # Tracking target
                self.target_search_mode = target_track.TRACK
                if range_data.range <= self.target_acquire_distance and range_data.range >= range_data.min_range:
                    # If we get close enough, stop moving
                    self.target_search_mode = target_track.ACQUIRED
            elif self.previous_target_search_mode == target_track.TRACK:
                # First frame target lost
                self.target_search_mode = target_track.LOST
                self.target_lost_time = rospy.get_time()

            elif self.previous_target_search_mode == target_track.LOST and \
                (rospy.get_time() - self.target_lost_time >= self.target_lost_timeout):
                # We've lost the target for long enough, go back to target search
                self.target_search_mode = target_track.SEARCH
            else:
                # Maintain current state
                self.target_search_mode = self.previous_target_search_mode

            #------------------------------------------------------------------------
            # Do state logic
            if self.target_search_mode == target_track.SEARCH:
                twist = self.target_search_logic()
            elif self.target_search_mode == target_track.TRACK:
                twist = self.target_track_logic( target_state )
            elif self.target_search_mode == target_track.LOST:
                twist = self.target_lost_logic()
            else: # Target already acquired
                twist = self.target_found_logic()

            # Send out the commanded twist
            self.pub.publish( twist )

            # Send out the target track states
            track = target_track()
            track.header.stamp = rospy.Time.now()
            track.tracking_state = self.target_search_mode
            track.tracking_state_prev = self.previous_target_search_mode
            track.tgt_pos_in_frame.x = self.azimuth_position
            track.tgt_pos_in_frame.y = self.elevation_position
            self.pub_track.publish( track )

            # Send out the range sensor state
            self.client.send_goal( self.range_state )

            # Package target search mode history again
            self.previous_target_search_mode = self.target_search_mode

            rate.sleep()

    #======================================================
    def target_search_logic( self ):
        """
        Spin the robot until it finds the target.
        Start spinning counter-clockwise (+z), until we have lost a target,
        then spin the direction we last saw a target
        """
        # Still looking for the target. Don't bother pinging the range sensor
        self.range_state.state = SensorStateGoal.INACTIVE

        tw = self.init_twist()
        tw.twist.angular.z = self.max_velocity / self.angular_moment * self.target_az_side # target_az_side sets direction (+/-z)
        return tw

    #======================================================
    def target_track_logic( self, target_state ):
        '''
        Move the robot in the direction of the identified target
        '''
        tw = self.init_twist()

        tw.twist.linear.x = self.max_velocity # Command it to walk forward towards the target

        # Spin until we center the target in frame
        half_image_width = target_state.camera_width / 2
        half_image_height = target_state.camera_height / 2
        self.azimuth_position = ( half_image_width - target_state.target_position.x ) / half_image_width # Left half of image is positive
        self.elevation_position = ( half_image_height - target_state.target_position.y ) / half_image_height
        self.target_az_side = np.sign( self.azimuth_position ) # Track which side the target was last seen on

        # If target is not centered in frame, add some spin to the gait
        if abs( self.azimuth_position ) > self.target_centered_tolerance:
            tw.twist.angular.z += self.azimuth_position * self.spin_gain

        # If the seeker isn't pointing straight forward, spin to try and center the seeker
        if abs( self.seeker_states.position[0] ) > self.seeker_turned_tolerance:
            tw.twist.angular.z += self.seeker_states.position[0] * self.seeker_az_gain

        # Tell the ultrasonic range sensor to turn on
        self.range_state.state = SensorStateGoal.ACTIVE

        tw = self.scale_twist( tw )

        return tw

    #======================================================
    def target_lost_logic( self ):
        '''
        Move in the same direction you last saw the target
        '''
        # Lost the target. Don't bother pinging the range sensor until we find it again
        self.range_state.state = SensorStateGoal.INACTIVE

        tw = self.init_twist()
        tw.twist.linear.x = self.max_velocity # Command it to walk forward, hoping the target is still there
        tw.twist.angular.z = 0.2 * self.target_az_side # rad/s
        tw = self.scale_twist( tw )
        return tw

    #======================================================
    def target_found_logic( self ):
        '''
        When the target has been acquired, stop moving
        '''
        # Found the target. we can stop pinging the range sensor
        self.range_state.state = SensorStateGoal.INACTIVE

        return self.init_twist()

    #======================================================
    def grab_target_states( self, target_states ):
        """
        Grabs target_states data from image processing topic to store locally
        """
        self.target_states = copy.deepcopy( target_states )

    #======================================================
    def grab_range_states( self, range_data ):
        """
        Grabs range_data data from ultrasonic range sensor topic to store locally
        """
        self.range_data = copy.deepcopy( range_data )

    #======================================================
    def grab_seeker_states( self, seeker_states ):
        """
        Grabs seeker_states data from seeker servo controller topic to store locally
        """
        self.seeker_states = copy.deepcopy( seeker_states )

    #======================================================
    def scale_twist( self, tw ):
        '''
        Makes sure the twist magnitude is not too large
        '''
        linear_velocity_mag = np.linalg.norm( [ tw.twist.linear.x,
                                                tw.twist.linear.y,
                                                tw.twist.linear.z] )
        angular_velocity_mag = np.linalg.norm( [tw.twist.angular.x,
                                                tw.twist.angular.y,
                                                tw.twist.angular.z] ) * self.angular_moment
        scaled_velocity_mag = self.max_velocity / ( linear_velocity_mag + angular_velocity_mag )

        tw.twist.linear.x *= scaled_velocity_mag
        tw.twist.linear.y *= scaled_velocity_mag
        tw.twist.linear.z *= scaled_velocity_mag
        tw.twist.angular.x *= scaled_velocity_mag
        tw.twist.angular.y *= scaled_velocity_mag
        tw.twist.angular.z *= scaled_velocity_mag

        return tw

    #======================================================
    def init_twist( self ):
        '''
        Initialize twist vector
        '''
        tw = TwistStamped()
        tw.header.stamp = rospy.Time.now()
        tw.header.frame_id = frame.ground

        # Zero out the twist command for now
        tw.twist.linear.x = 0.0
        tw.twist.linear.y = 0.0
        tw.twist.linear.z = 0.0
        tw.twist.angular.x = 0.0
        tw.twist.angular.y = 0.0
        tw.twist.angular.z = 0.0
        return tw

#==============================================================================
if __name__ == "__main__":
    try:
        wc = walk_control_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
