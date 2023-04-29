#!/usr/bin/env python3
import os
import rospy

class frames:
    def __init__( self, ns="frames" ):
        #==============================================================
        # Strings defining acceptable tf2 transformation frame names
        #==============================================================
        self.body = rospy.get_param( os.path.join( ns, "body/body" ), default="body/body" ) # Robot body frame, defined at base of robot hardware.
        # Intermediate body frames
        self.body_commanded = rospy.get_param( os.path.join( ns, "body/commanded" ), default="body/commanded" )
        self.body_filtered = rospy.get_param( os.path.join( ns, "body/filtered" ), default="body/filtered" )
        self.body_gravity_adjust = rospy.get_param( os.path.join( ns, "body/gravity_adjust" ), default="body/gravity_adjust" )

        self.imu = rospy.get_param( os.path.join( ns, "imu" ), default="imu" ) # IMU sensor frame, mounted to robot body

        # Seeker-related frames
        self.neck_static = rospy.get_param( os.path.join( ns, "seeker/neck_static" ), default="seeker/neck_static" ) # Static location where the seeker az joint mounts
        self.neck = rospy.get_param( os.path.join( ns, "seeker/neck" ), default="seeker/neck" ) # z axis where seeker azimuth rotates
        self.seeker  = rospy.get_param( os.path.join( ns, "seeker/seeker"  ), default="seeker/seeker" ) # x points out from seeker
        self.camera  = rospy.get_param( os.path.join( ns, "seeker/camera"  ), default="seeker/camera" ) # z points out from the camera
        self.ultrasonic  = rospy.get_param( os.path.join( ns, "seeker/ultrasonic"  ), default="seeker/ultrasonic" ) # x points out from the range sensor

        # Each leg gets its own transform frames
        self.shoulder = rospy.get_param( os.path.join( ns, "legs/shoulder" ) )
        self.hip = rospy.get_param( os.path.join( ns, "legs/hip" ) )
        self.coxa = rospy.get_param( os.path.join( ns, "legs/coxa" ) )
        self.knee = rospy.get_param( os.path.join( ns, "legs/knee" ) )
        self.femur = rospy.get_param( os.path.join( ns, "legs/femur" ) )
        self.ankle = rospy.get_param( os.path.join( ns, "legs/ankle" ) )
        self.tibia = rospy.get_param( os.path.join( ns, "legs/tibia" ) )
        self.foot = rospy.get_param( os.path.join( ns, "legs/foot" ) )

        self.ground = rospy.get_param( os.path.join( ns, "ground/ground" ), default="ground" ) # Ground frame, placed under robot body. Walking gait calculates in this frame. Child of body
        self.foot_ground = rospy.get_param( os.path.join( ns, "ground/foot" ) ) # Foot frame referenced to the ground
