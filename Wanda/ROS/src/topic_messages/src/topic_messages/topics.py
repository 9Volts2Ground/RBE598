#!/usr/bin/env python3
import rospy

#---------------------------------------------------
# Grab global parameters
NUM_LEGS = rospy.get_param( 'num_legs', default=6 )

# Defines acceptable strings for ROS topics
class topics:
    def __init__(self):

        #=================================================================
        # Kinematic topics
        #=================================================================
        # Leg topics
        self.leg_foot_states = []
        self.leg_joint_states = []
        header = rospy.get_param( "topics/leg/header", default="leg" )
        for leg in range( NUM_LEGS ):
            topic = rospy.get_param( f"topics/{header}/foot_states", default="foot_states" )
            self.leg_foot_states.append( f"{header}{leg}/{topic}" )
            topic = rospy.get_param( f"topics/{header}/joint_states", default="joint_states" )
            self.leg_joint_states.append( f"{header}{leg}/{topic}")

        # Seeker topics
        header = rospy.get_param( "topics/seeker/header", default="seeker" )
        topic = rospy.get_param( f"topics/{header}/nominal", default="nominal" )
        self.seeker_nominal = f"{header}/{topic}"
        topic = rospy.get_param( f"topics/{header}/search", default="search" )
        self.seeker_search = f"{header}/{topic}"
        topic = rospy.get_param( f"topics/{header}/filtered", default="filtered" )
        self.seeker_filtered = f"{header}/{topic}"

        # Body Pose topics
        header = rospy.get_param( "topics/body_pose/header", default="body_pose" )
        topic = rospy.get_param( f"topics/{header}/commanded", default="commanded" )
        self.body_pose_commanded = f"{header}/{topic}"
        topic = rospy.get_param( f"topics/{header}/gravity_adjusted", default="gravity_adjusted" )
        self.body_pose_gravity_adjusted = f"{header}/{topic}"
        topic = rospy.get_param( f"topics/{header}/filtered", default="filtered" )
        self.body_pose_filtered = f"{header}/{topic}"

        # Walking direction twist topics
        header = rospy.get_param( "topics/walking_twist/header", default="walking_twist" )
        topic = rospy.get_param( f"topics/{header}/commanded", default="commanded" )
        self.walking_twist_commanded = f"{header}/{topic}"
        topic = rospy.get_param( f"topics/{header}/filtered", default="filtered" )
        self.walking_twist_filtered = f"{header}/{topic}"
        topic = rospy.get_param( f"topics/{header}/uncertainty", default="uncertainty" )
        self.walking_twist_uncertainty = f"{header}/{topic}"

        # Walking gait info
        header = rospy.get_param( "topics/gait_state/header", default="gait_state" )
        topic = rospy.get_param( f"topics/{header}/gait_state", default="gait_state" )
        self.gait_state = f"{header}/{topic}"


        #=================================================================
        # Sensor topics
        #=================================================================
        # Camera topics
        header = rospy.get_param( "topics/camera/header", default="camera" )
        topic = rospy.get_param( f"topics/{header}/image_raw", default="image_raw")
        self.camera_image_raw = f"{header}/{topic}"
        topic = rospy.get_param( f"topics/{header}/camera_info", default="camera_info")
        self.camera_info = f"{header}/{topic}"
        topic = rospy.get_param( f"topics/{header}/target_track", default="target_track")
        self.camera_target_track = f"{header}/{topic}"

        # IMU topics
        header = rospy.get_param( "topics/imu/header", default="imu" )
        topic = rospy.get_param( f"topics/{header}/data_raw", default="data_raw" )
        self.imu_data_raw = f"{header}/{topic}"
        topic = rospy.get_param( f"topics/{header}/data_filtered", default="data_filtered" )
        self.imu_data_filtered = f"{header}/{topic}"
        topic = rospy.get_param( f"topics/{header}/orientation_cov", default="orientation_cov" )
        self.imu_orientation_cov = f"{header}/{topic}"

        # Range sensor topics
        header = rospy.get_param( "topics/range/header", default="range" )
        topic = rospy.get_param( f"topics/{header}/data_raw", default="data_raw")
        self.range_data_raw = f"{header}/{topic}"
        topic = rospy.get_param( f"topics/{header}/point_cloud", default="point_cloud")
        self.range_point_cloud = f"{header}/{topic}"
        topic = rospy.get_param( f"topics/{header}/filtered", default="filtered")
        self.range_filtered = f"{header}/{topic}"

        # Other sensor related topics to publish
        header = rospy.get_param( "topics/temperature/header", default="temperature")
        topic = rospy.get_param( f"topics/{header}/cpu", "cpu")
        self.cpu_temp = f"{header}/{topic}"

        #=================================================================
        # Odometry topics
        #=================================================================
        header = rospy.get_param( "topics/odometry/header", default="odometry" )
        topic = rospy.get_param( f"topics/{header}/ekf", default="ekf" )
        self.odometry_ekf = f"{header}/{topic}"

        #=================================================================
        # Target Track topics
        #=================================================================
        header = rospy.get_param( "topics/target_track/header", default="target_track" )
        topic = rospy.get_param( f"topics/{header}/image_states", default="image_states" )
        self.target_image_track = f"{header}/{topic}"
        topic = rospy.get_param( f"topics.{header}/track_mode", default="track_mode" )
        self.target_track_mode = f"{header}/{topic}"


#==============================================================================
if __name__ == '__main__':
    top = topics()
    print("Done")
