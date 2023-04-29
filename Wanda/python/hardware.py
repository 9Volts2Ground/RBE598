import cv2
import numpy as np
import os
from rotation import rotrx, rotry, rotrz
import subprocess
import time

class hardware:

    # Make this class a singleton, so we only turn on the PWM channels once
    __instance = None

    def __new__(cls, *args, **kwargs):
        if not hardware.__instance:
            hardware.__instance = object.__new__(cls)
        return hardware.__instance

    #==========================================================================
    def __init__(self):

        # Pose of body relative to inertial
        # [0:3] = linear position
        # [3:6] = linear velocity
        # [6:9] = linear acceleration
        # [9:12] = orientation (yaw, pitch, roll)
        # [12:15] = angular velocity
        self.body_state_inertial = np.zeros( 15 )

        # Check to see if we are running on Wanda or another machine
        if "wanda" in os.getcwd():
            self.wanda = True

            # If running Wanda hardware, init hardware-specific classes
            from Servo import Servo
            self.servo = Servo()

            from Led import Led
            self.led = Led()

            # Set up camera parameters
            self.camera_resolution = [2560, 1440] # width, height
            self.camera = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
            time.sleep(0.05)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_resolution[0])
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_resolution[1])

            # Set up ultrasonic sensor
            from Ultrasonic import Ultrasonic
            self.sonic = Ultrasonic()

            # Check the CPU temp right away
            self.cpu_temp = self.get_cpu_temp()
            if self.cpu_temp > 60:
                print("WARNING: high CPU temps: ", str(self.cpu_temp) )
        else:
            self.wanda = False

        #======================================================================
        # Set up IMU sensor
        from IMU import IMU
        self.IMU = IMU( self.wanda )
        # self.body_state_inertial[9:12] = self.IMU.euler_angles

        #======================================================================
        # Leg parameters
        #======================================================================

        # Vector from body center to hip joints, m
        self.s = np.array([ [ -0.055, 0.076162, 0.048 ],        # Front left
                            [  0.055, 0.076162, 0.048 ],        # Front right
                            [ -0.083376, 0.0, 0.048 ],         # Middle left
                            [  0.083376, 0.0, 0.048 ],         # Middle right
                            [ -0.055, -0.076162, 0.048 ],       # Back left
                            [  0.055, -0.076162, 0.048 ] ] ).T  # Back right

        # Leg link lengths, m
        self.L1 = 0.03226     # Coxa
        self.L2 = 0.090      # Femur
        self.L3 = 0.113     # Tibia

        # PWM channel for each motor
        self.legChannel = np.array( [ [ 16, 17, 18 ],       # Front left
                                      [ 15, 14, 13 ],       # Front right
                                      [ 19, 20, 21 ],       # Middle left
                                      [ 12, 11, 10 ],       # Middle right
                                      [ 22, 23, 27 ],       # Back left
                                      [  9,  8, 31 ] ] ).T  # Back right

        # Motor angle centers, degrees
        self.motorCenter = np.array( [  [ 115, 100, 160 ],
                                        [ 120, 105, 20 ],
                                        [ 120, 110, 155 ],
                                        [ 120, 110, 26 ],
                                        [ 100, 110, 143 ],
                                        [ 125, 120, 25 ] ] ).T

        # Motor angle mins, degrees
        self.motorMin = np.array( [ [ 50, 20, 0 ],
                                    [ 85, 10, 0 ],
                                    [ 70, 15, 0 ],
                                    [ 70, 15, 0 ],
                                    [ 60, 20, 0 ],
                                    [ 60, 20, 0 ] ] ).T

        # Motor angle maxs, degrees
        self.motorMax = np.array( [ [ 150, 180, 180 ],
                                    [ 175, 180, 180 ],
                                    [ 160, 180, 180 ],
                                    [ 170, 180, 180 ],
                                    [ 160, 180, 180 ],
                                    [ 165, 180, 180 ] ] ).T

        # Motor angle command matches matches defined orientation angle
        self.motorOrientation = np.array( [ [ -1, -1, -1 ],
                                            [ -1,  1,  1 ],
                                            [ -1, -1, -1 ],
                                            [ -1,  1,  1 ],
                                            [ -1, -1, -1 ],
                                            [ -1,  1,  1 ] ] ).T

        self.alpha_offset = np.array( [ np.arctan2( self.s[1,0], self.s[0,0] ),
                                        np.arctan2( self.s[1,1], self.s[0,1] ),
                                        np.arctan2( self.s[1,2], self.s[0,2] ),
                                        np.arctan2( self.s[1,3], self.s[0,3] ),
                                        np.arctan2( self.s[1,4], self.s[0,4] ),
                                        np.arctan2( self.s[1,5], self.s[0,5] ) ] )

        # Current state of motor joint angles, radians. Just init to 0's, to be filled in by algorithms later
        self.joint_angles = np.zeros( shape = (3,6) )

        # Current state of foot position, relative to body frame
        self.foot_position = np.zeros( shape = (3,6) )
        self.foot_position_inertial = np.zeros( shape = (3,6) )

        # Flag if foot is on the ground or in the air
        self.foot_off_ground = np.zeros( 6 )

        # Mapping between leg and LED index
        self.leg2led = [6, 0, 5, 1, 4, 2]

        # Tracks LED for each leg
        self.led_color = np.zeros( shape = (3,6) )

        #======================================================================
        # Seeker parameters
        # All arrays are ordered az, el
        #======================================================================

        # Camera parameters
        self.focal_length = 3.6 # mm

        # Transformation from body frame to el motor axis
        self.T_body2neck = np.array( [ [0.0, -1.0, 0.0, 0.0],
                                       [1.0,  0.0, 0.0, self.s[1,0]],
                                       [0.0,  0.0, 1.0, 0.13858],
                                       [0.0,  0.0, 0.0, 1.0] ] )
        self.T_body2cam = np.identity(4)

        self.seeker_extension_length = 0.03226 # Length of extension from el motor axis to camera face

        # Seeker angles, radians, just init to 0
        self.seeker_angles = np.array( [0.0, 0.0] )

        self.seeker_channel = np.array( [1, 0] )

        self.seekerCenter = np.array( [ 85, 91 ] )
        self.seekerMin = np.array( [ 20, 75 ] )
        self.seekerMax = np.array( [ 160, 180 ] )

        self.seekerOrientation = np.array( [1, 1] )

        self.seeker_led = 3 # LED array number
        self.seeker_color = np.zeros(3)

        self.seeker_actuated_time = time.time()


    #==========================================================================
    def init_angles(self):
        """Curl Wanda up to start with"""
        self.joint_angles = [ [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ],
                              [ np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2 ],
                              [ np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2 ] ]

        # If we are running on real hardware, make her move to these angles
        if self.wanda:
            self.move_all_joints( self )
            self.move_seeker( self ) # Just move to initial 0,0

    #==========================================================================
    def move_all_joints( self ):
        """ Command all motors to move to joint angles defined in self.joint_angles """

        # Only try to move the motors if we are running on real hardware
        if self.wanda:

            # Loop through all legs and joints
            for leg in range(6):
                for joint in range(3):

                    # Convert angle to degrees, offset it by the hardware 0 value
                    angle_command = self.joint_angles[joint,leg] * 180/np.pi * self.motorOrientation[joint,leg] + self.motorCenter[joint,leg]

                    # Saturate the motor commands to min/max values
                    if angle_command > self.motorMax[joint,leg]:
                        angle_command = self.motorMax[joint,leg]
                    elif angle_command < self.motorMin[joint,leg]:
                        angle_command = self.motorMin[joint,leg]

                    # Ensure angle is an int to pass into setServoAngle()
                    angle_command = int( angle_command )

                    self.servo.setServoAngle( self.legChannel[joint, leg], angle_command )

                    # Update LED to show gait state------------------
                    if self.foot_off_ground[leg]:
                        self.led_color[:,leg] = [0, 0, 10]
                    else:
                        self.led_color[:,leg] = [0, 10, 0]

                    try:
                        self.led.setColor(self.leg2led[leg], self.led_color[:,leg].tolist() )
                    except:
                        pass
    #==========================================================================
    def move_seeker( self ):
        """Move where the seeker is pointed"""
        self.seeker_actuated_time = time.time()
        if self.wanda:

            # Loop through both az and el seeker servos
            for motor in range(2):
                angle_command = self.seeker_angles[motor] * 180/np.pi * self.seekerOrientation[motor] + self.seekerCenter[motor]

                # Saturate the motor commands to min/max values
                if angle_command > self.seekerMax[motor]:
                    angle_command = self.seekerMax[motor]
                elif angle_command < self.seekerMin[motor]:
                    angle_command = self.seekerMin[motor]

                # Ensure angle is an int to pass into setServoAngle()
                angle_command = int( angle_command )

                self.servo.setServoAngle( self.seeker_channel[motor], angle_command )

        self.update_seeker_orientation()

    #==========================================================================
    def update_seeker_orientation( self ):
        """
        Calculate transformation matrix from body to camera frame given seeker az, el angles
        """
        T_neck2az = np.array( [ [np.cos(self.seeker_angles[0]), 0.0,  np.sin( self.seeker_angles[0]), 0.0],
                                [np.sin(self.seeker_angles[0]), 0.0, -np.cos( self.seeker_angles[0]), 0.0],
                                [0.0, 1.0, 0.0, 0.0],
                                [0.0, 0.0, 0.0, 1.0] ] )

        T_az2cam  = np.array( [ [np.cos(self.seeker_angles[1]), 0.0, -np.sin( self.seeker_angles[1]), np.cos( self.seeker_angles[1])*self.seeker_extension_length],
                                [np.sin(self.seeker_angles[1]), 0.0,  np.cos( self.seeker_angles[1]), np.sin( self.seeker_angles[1])*self.seeker_extension_length],
                                [0.0, -1.0, 0.0, 0.0],
                                [0.0,  0.0, 0.0, 1.0] ] )

        self.T_body2cam = self.T_body2neck @ T_neck2az @ T_az2cam


    #==========================================================================
    def capture_image( self ):
        """
            Capture image with camera
        Returns:
            returnd (bool): did the camera capture an image?
            image: array of pixels
        """

        # Set the camera LED to red when taking photo
        led_pre_camera = self.seeker_color
        self.seeker_color = [10, 0, 0]
        try:
            self.led.setColor( self.seeker_led, self.seeker_color )
        except:
            pass

        if self.wanda: # Only take pic if running on Wanda hardware with camera
            returned, image = self.camera.read()
            image = cv2.flip( image, 0 ) # The seeker is mounted upside down, need to flip the image

        # Put the seeker LED back as before
        self.seeker_color = led_pre_camera
        try:
            self.led.setColor( self.seeker_led, self.seeker_color)
        except:
            pass

        return returned, image

    #==========================================================================
    # Changes camera resolution
    # def capture_image( self ):

    #==========================================================================
    def transform_body2shoulder( self, leg ):
        """
        Args:
            leg (integer): Which leg to get transformation for
        Returns:
            T_body2shoulder [4x4 np array]: Transformation from body frame to shoulder joint of leg
        """

        T_body2shoulder = np.array( [[ np.cos(self.alpha_offset[leg]), -np.sin(self.alpha_offset[leg]), 0, self.s[0,leg] ],
                                     [ np.sin(self.alpha_offset[leg]),  np.cos(self.alpha_offset[leg]), 0, self.s[1,leg] ],
                                     [ 0.0, 0.0, 1.0, self.s[2,leg] ],
                                     [ 0.0, 0.0, 0.0, 1.0 ] ] )
        return T_body2shoulder

    #==========================================================================
    def transform_shoulder2foot( self, leg ):
        """Calculates 4x4 transformation from given shoulder joint to the foot
        Args:
            leg (int): Which leg to calculate transformation for
        Returns:
            [4x4 numpy array]: Transformation matrix from shoulder joint to foot
        """
        A1 = np.array([ [np.cos(self.joint_angles[0,leg]), 0.0,  np.sin(self.joint_angles[0,leg]), self.L1*np.cos(self.joint_angles[0,leg])],
                        [np.sin(self.joint_angles[0,leg]), 0.0, -np.cos(self.joint_angles[0,leg]), self.L1*np.sin(self.joint_angles[0,leg])],
                        [0.0, 1.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0] ] )

        A2 = np.array([ [np.cos(self.joint_angles[1,leg]), -np.sin(self.joint_angles[1,leg]), 0.0, self.L2*np.cos(self.joint_angles[1,leg])],
                        [np.sin(self.joint_angles[1,leg]),  np.cos(self.joint_angles[1,leg]), 0.0, self.L2*np.sin(self.joint_angles[1,leg])],
                        [0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0] ] )

        A3 = np.array([ [np.cos(self.joint_angles[2,leg]), -np.sin(self.joint_angles[2,leg]), 0.0, self.L3*np.cos(self.joint_angles[2,leg])],
                        [np.sin(self.joint_angles[2,leg]),  np.cos(self.joint_angles[2,leg]), 0.0, self.L3*np.sin(self.joint_angles[2,leg])],
                        [0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0] ] )

        return A1 @ A2 @ A3

    #==========================================================================
    def fkine_body2foot( self, leg ):
        return self.transform_body2shoulder( leg ) @ self.transform_shoulder2foot( leg )

    #==========================================================================
    def update_foot_position_body_frame( self ):

        for leg in range(6):

            Tbf = self.fkine_body2foot( leg )

            self.foot_position[:,leg] = Tbf[0:3,3]

    #==========================================================================
    def leg_vector2foot_position( self, Leng, n):

        for leg in range(6):
            self.foot_position[:,leg] = Leng[leg] * n[:,leg]

        # # Invert transformation from inertial to body frame
        # T_i2b = self.transform_inertial2body()
        # R_i2b_T = T_i2b[0:3,0:3].T

        # T_b2i = np.identity(4)
        # T_b2i[0:3,0:3] = R_i2b_T
        # T_b2i[0:3,3] = -R_i2b_T @ T_i2b[0:3,3].T

        # for leg in range(6):
        #     self.foot_position_inertial[:,leg] = n[:,leg] * Leng[leg]
        #     foot_pos = T_b2i @ np.array( np.append( self.foot_position_inertial[:,leg], 1) ).T
        #     self.foot_position[:,leg] = foot_pos[0:3]


    #==========================================================================
    def rotation_inertial2body( self ):
        """Uses individual Euler angles, calculates rotation matrix between inertial and body frames.
        From body frame: rotate yaw, pitch roll
        """

        R_roll = rotry( self.body_state_inertial[11] )
        R_pitch = rotrx( self.body_state_inertial[10] )
        R_yaw = rotrz( self.body_state_inertial[9])

        return R_roll @ R_pitch @ R_yaw # Body orientation matrix in inertial frame

    #==========================================================================
    def transform_inertial2body( self ):
        """
        Calculates transformation matrix from inertial to body frame
        Returns: T_inertial2body [4x4 numpy array]
        """
        T_inertial2body = np.identity(4)
        T_inertial2body[0:3,0:3] = self.rotation_inertial2body()
        T_inertial2body[0:3,3] = self.body_state_inertial[0:3]

        return T_inertial2body

    #==========================================================================
    def update_foot_position_inertial( self ):
        """
        Given body pose in inertial frame and foot positions in body frame,
        updates foot positions in inertial frame
        """
        T_inertial2body = self.transform_inertial2body()
        for leg in range(6):
            foot_inertial = T_inertial2body @ np.append( self.foot_position[:,leg], 1.0).T
            self.foot_position_inertial[:,leg] = foot_inertial[0:3]

    #==========================================================================
    def transformation_inverse( self, T_in ):
        """
        Calculates inverse of 4x4 transformation matrix
        Args:
            T_in [4x4 numpy array]: Transformation matrix to invert
        Returns:
            T_out [4x4 numpy array]: Inverted transformation matrix
        """

        R_in_T = T_in[0:3,0:3].T

        T_out = np.identity(4)
        T_out[0:3,0:3] = R_in_T
        T_out[0:3,3] = -R_in_T @ T_in[0:3,3].T

        return T_out

    #==========================================================================
    def get_range( self ):
        range, returned = self.sonic.get_distance()
        return range

    #==========================================================================
    def average_range( self, num_measurements ):
        """
        Sends n pulses, takes average and standard deviation of return
        Returns:
            [type]: [description]
        """
        if self.wanda:
            # Set the sonic LED to blue when measuring range
            led_pre_sonic = self.seeker_color
            self.seeker_color = [0, 0, 10]
            try:
                self.led.setColor( self.seeker_led, self.seeker_color )
            except:
                pass

            range, std = self.sonic.average_distances( num_measurements )

            # Put the seeker LED back as before
            self.seeker_color = led_pre_sonic
            try:
                self.led.setColor( self.seeker_led, self.seeker_color)
            except:
                pass

            return range, std
        else:
            return 0.0, 0.0

    #==========================================================================
    def get_cpu_temp( self ):
        """Returns CPU temperature as a float"""
        out = subprocess.Popen(['cat', '/sys/class/thermal/thermal_zone0/temp'], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        stdout, stderr = out.communicate()
        temp_string = stdout[0:5]
        temp_float = float(temp_string)/1000.0
        return(temp_float)

