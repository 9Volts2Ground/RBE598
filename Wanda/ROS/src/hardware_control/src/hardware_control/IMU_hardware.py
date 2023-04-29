#!/usr/bin/env python3
import numpy as np
import rospy

# Only needed for non-hardware sensor modeling
import tf2_ros
from tf.transformations import quaternion_matrix

# Custom libraries
from system_globals import frames

#================================================
# Get global parameters
DEG2RAD = np.pi/180
GRAVITY = 9.81 # m/s**2

#----------------------------------------
# Set global flags and class instances
frame = frames.frames()

#==============================================================================
class IMU_hardware:
    # Make this class a singleton, so we only turn on the PWM channels once
    __instance = None
    def __new__(cls, *args, **kwargs):
        if not IMU_hardware.__instance:
            IMU_hardware.__instance = object.__new__(cls)
        return IMU_hardware.__instance

    #==========================================================================
    def __init__(self,
                 wanda = False,
                 acc_stddev = 0.1,
                 gyro_stddev = 0.1 ):
        self.wanda = wanda
        if self.wanda:
            # Set up PWM channel
            from mpu6050 import mpu6050 # https://github.com/m-rtijn/mpu6050
            self.sensor = mpu6050( address=0x68, bus=1 )
            self.sensor.set_accel_range( mpu6050.ACCEL_RANGE_2G ) # Acc can be +/- 2, 4, 8, or 16 g
            self.sensor.set_gyro_range( mpu6050.GYRO_RANGE_250DEG ) # Gyro can be +/- 250, 500, 1k, or 2k deg/s
        else:
            self.acc_stddev = acc_stddev
            self.gyro_stddev = gyro_stddev

            # We need to apply artificial gravity compensation. Make sure we know the orientation relative to the ground
            self.tfBuffer = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener( self.tfBuffer )

            self.T_imu2ground = np.eye(4)

        self.accel_measured = np.zeros( 3 )
        self.gyro_measured = np.zeros( 3 )

        self.accel_offset = np.zeros( 3 )
        self.gyro_offset = np.zeros( 3 )

    #==========================================================================
    def get_current_measurement( self, twist=None ):

        success = True
        if self.wanda:
            success = self.hardware_measurements()
        else:
            success = self.emulated_measurements( twist )

        if success:
            # Un-bias the sensor with the calibrated offset values
            self.accel_measured -= self.accel_offset
            self.gyro_measured -= self.gyro_offset

        return success

    #==========================================================================
    def hardware_measurements( self ):
        success = True
        try:
            accel_data = self.sensor.get_accel_data()
            gyro_data = self.sensor.get_gyro_data()

            self.accel_measured = np.array( [ accel_data['y'], -accel_data['x'], accel_data['z'] ] )
            self.gyro_measured  = np.array( [ gyro_data['y'], -gyro_data['x'], gyro_data['z'] ] ) * DEG2RAD

        except:
            rospy.logwarn("Can't update IMU data from hardware")
            success = False
        return success

    #==========================================================================
    def emulated_measurements( self, twist ):
        # When not testing on hardware, return random values
        self.accel_measured = np.random.normal( 0.0, scale=self.acc_stddev, size=(3) ) # gauss distribution, m/s**2
        self.accel_measured[2] += GRAVITY # Let's pretend there's some gravity straight up

        # Rotate the gravity signal into the right frame
        try:
            tran = self.tfBuffer.lookup_transform( frame.imu, frame.ground, rospy.Time(0) )

            q = [tran.transform.rotation.x,
                 tran.transform.rotation.y,
                 tran.transform.rotation.z,
                 tran.transform.rotation.w]

            self.T_imu2ground = np.array( quaternion_matrix( q ) )
            self.T_imu2ground[:3,3] = [tran.transform.translation.x,
                                        tran.transform.translation.y,
                                        tran.transform.translation.z]

        except:
            rospy.logwarn_throttle( 10, f"Could not get T_imu2ground in {rospy.get_name()}. Using previous transformation")

        self.accel_measured[:3] = self.T_imu2ground[:3,:3] @ self.accel_measured[:3].T

        self.gyro_measured = np.random.normal( 0.0, scale=self.gyro_stddev, size=(3) ) # rad/s

        # Add in our rotational command
        angular_velocity = [twist.twist.angular.x,
                            twist.twist.angular.y,
                            twist.twist.angular.z]
        self.gyro_measured += self.T_imu2ground[:3,:3] @ angular_velocity[:3]

        return True