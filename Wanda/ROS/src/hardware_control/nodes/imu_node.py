#!/usr/bin/env python3
import actionlib
import copy
import numpy as np
import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu

# Custom libraries
from hardware_control import IMU_hardware
from hardware_control.msg import ImuCalibrateAction
from system_globals import frames
from topic_messages import topics

#================================================
# Get global parameters
GRAVITY = 9.81 # m/s**2
MAX_G = 2 * GRAVITY
MAX_GYRO = 250 * np.pi/180

#----------------------------------------
# Set global flags and class instances
frame = frames.frames()
top = topics.topics()

#==========================================================================
class imu_node:
    def __init__(self):
        rospy.init_node( "imu_node", anonymous=False )
        self.pub = rospy.Publisher( top.imu_data_raw, Imu, queue_size=1 )

        self.hardware_rate = rospy.get_param( 'imu/rate', default=50 )
        self.rate = rospy.Rate( self.hardware_rate ) # Hz, double check hardware spec

        # Pull in user parameters
        self.calibrate_active = rospy.get_param( 'imu/calibrate', default=False )

        # Read in how long we want to pull the sensor for to set the calibration
        self.calibration_duration = rospy.get_param( "imu/calibration_duration", default=0.1 )

        # How much standard deviation we expect the IMU to produce
        self.acc_stddev = rospy.get_param( 'imu/acc_stddev', default=0.1 )
        self.gyro_stddev = rospy.get_param( 'imu/gyro_stddev', default=0.1 )

        # Initialize IMU hardware class
        IS_WANDA = rospy.get_param( 'IS_WANDA', default=False )
        self.imu_hrd = IMU_hardware.IMU_hardware( IS_WANDA,
                                                  acc_stddev = self.acc_stddev,
                                                  gyro_stddev = self.gyro_stddev )

        # If we are emulating the data, grab our velocity commands
        self.twist = TwistStamped()
        if not IS_WANDA:
            self.twist.twist.linear.x = 0.0
            self.twist.twist.linear.y = 0.0
            self.twist.twist.linear.z = 0.0
            self.twist.twist.angular.x = 0.0
            self.twist.twist.angular.y = 0.0
            self.twist.twist.angular.z = 0.0
            rospy.Subscriber( top.walking_twist_filtered,
                              TwistStamped,
                              self.get_walking_twist )

        # Initialize IMU topic message
        self.imu_topic = Imu()
        self.imu_topic.header.frame_id = frame.imu

        # This IMU does not return absolute orientation.
        self.imu_topic.orientation_covariance[0] = -1

        # If we are running with the simulation, wait for the clock to turn on
        while rospy.get_time() == 0 and not rospy.is_shutdown():
            rospy.loginfo_once( "imu_node.py waiting for clock time to be published")

        # Set up calibration parameters and
        self.measurement_count = 0
        self.calibration_start_time = rospy.get_time()
        self.accel_measures = np.zeros(3)
        self.gyro_measures = np.zeros(3)

        # Set up the calibration server
        self.server = actionlib.SimpleActionServer( 'imu_calibration_server',
                                                    ImuCalibrateAction,
                                                    self.calibration_action,
                                                    auto_start=False )
        self.server.start()

    #================================================
    def process_imu_data( self ):
        # While ROS is still running.
        while not rospy.is_shutdown():
            # Grab hardware data
            if self.imu_hrd.get_current_measurement( self.twist ):

                self.calibrate_imu()

                self.publish_data()

            self.rate.sleep()

    #================================================
    def publish_data( self ):

        self.imu_topic.linear_acceleration.x = self.imu_hrd.accel_measured[0]
        self.imu_topic.linear_acceleration.y = self.imu_hrd.accel_measured[1]
        self.imu_topic.linear_acceleration.z = self.imu_hrd.accel_measured[2]

        self.imu_topic.angular_velocity.x = self.imu_hrd.gyro_measured[0]
        self.imu_topic.angular_velocity.y = self.imu_hrd.gyro_measured[1]
        self.imu_topic.angular_velocity.z = self.imu_hrd.gyro_measured[2]

        self.calc_uncertainties()

        self.imu_topic.header.stamp = rospy.Time.now()
        self.pub.publish( self.imu_topic )

    #================================================
    def calc_uncertainties( self ):

        # If the sensor is saturating, make the uncertainty explode
        cov_index = [0, 4, 8]
        for axis in range( 3 ):
            # Set the accelerometer measurement covariance
            if abs( self.imu_hrd.accel_measured[axis] ) >= MAX_G:
                self.imu_topic.linear_acceleration_covariance[cov_index[axis]] = 5
            else:
                self.imu_topic.linear_acceleration_covariance[cov_index[axis]] = 0.005

            # Set the gyrometer measurement covariance
            if abs( self.imu_hrd.gyro_measured[axis] ) >= MAX_GYRO:
                self.imu_topic.angular_velocity_covariance[cov_index[axis]] = 5
            else:
                self.imu_topic.angular_velocity_covariance[cov_index[axis]] = 0.005

    #================================================
    def calibrate_imu( self ):

        if not self.calibrate_active:
            return

        calibrate_long_enough = rospy.get_time() - self.calibration_start_time >= self.calibration_duration

        if not calibrate_long_enough:
            self.measurement_count += 1
            self.accel_measures += self.imu_hrd.accel_measured
            self.gyro_measures += self.imu_hrd.gyro_measured
        else:
            self.calibrate_active = False
            self.imu_hrd.accel_offset += self.accel_measures / self.measurement_count
            self.imu_hrd.gyro_offset += self.gyro_measures / self.measurement_count

            # Get rid of gravity from the accelerometer
            self.imu_hrd.accel_offset[2] -= GRAVITY

            rospy.loginfo( f"Finished IMU calibration with {self.measurement_count} measurements. \n \
            Accel offset: {self.imu_hrd.accel_offset}. \n \
            Gyro offset: {self.imu_hrd.gyro_offset}" )

    #================================================
    def calibration_action( self, goal ):

        if goal.calibrate_imu:
            self.calibration_duration = goal.calibration_duration
            self.calibrate_active = True
            self.calibration_start_time = rospy.get_time()
            self.measurement_count = 0
            self.accel_measures = np.zeros(3)
            self.gyro_measures = np.zeros(3)

        self.server.set_succeeded()

    #================================================
    def get_walking_twist( self, twist ):
        self.twist = copy.deepcopy( twist )

#==============================================================================
if __name__ == '__main__':
    try:
        imu = imu_node()
        imu.process_imu_data()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
