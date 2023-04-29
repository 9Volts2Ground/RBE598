#ifndef __TOPICS_H__
#define __TOPICS_H__

#include <string>

// ROS libraries
#include <ros/ros.h>

//=============================================================================
class Topics
{

public:
    //---------------------------------------------------------------
    // Constructor
    Topics(
        ros::NodeHandle& nh
    )
    {
        set_topics( nh );
    }

    //---------------------------------------------------------------
    // Sets class member topic labels from global ROS parameters
    void set_topics(
        ros::NodeHandle& nh
    )
    {
        nh.param<std::string>("topics/leg/header", m_leg_header, "leg" );
        nh.param<std::string>("topics/" + m_leg_header + "/joint_states", m_leg_joint_states, "joint_states" );
        nh.param<std::string>("topics/" + m_leg_header + "/foot_states", m_leg_foot_states, "foot_states" );

        // Strings we use multiple times
        std::string header, topic;

        // Seeker topics
        std::string seeker_joint_states;
        nh.param<std::string>("topics/seeker/header", header, "seeker");
        nh.param<std::string>("topics/" + header + "/nominal", topic, "nominal");
        m_seeker_nominal = header + "/" + topic;
        nh.param<std::string>("topics/" + header + "/search", topic, "search");
        m_seeker_search = header + "/" + topic;
        nh.param<std::string>("topics/" + header + "/filtered", topic, "filtered");
        m_seeker_filtered = header + "/" + topic;

        // Body pose topics
        nh.param<std::string>("topics/body_pose/header", header, "body_pose");
        nh.param<std::string>("topics/" + header + "/commanded", topic, "commanded");
        m_body_pose_commanded = header + "/" + topic;
        nh.param<std::string>("topics/" + header + "/gravity_adjusted", topic, "gravity_adjusted");
        m_body_pose_gravity_adjusted = header + "/" + topic;
        nh.param<std::string>("topics/" + header + "/filtered", topic, "filtered");
        m_body_pose_filtered = header + "/" + topic;

        // Walking direction twist topics
        nh.param<std::string>("topics/walking_twist/header", header, "walking_twist");
        nh.param<std::string>("topics/" + header + "/commanded", topic, "commanded");
        m_walking_twist_commanded = header + "/" + topic;
        nh.param<std::string>("topics/" + header + "/filtered", topic, "filtered");
        m_walking_twist_filtered = header + "/" + topic;
        nh.param<std::string>("topics/" + header + "/uncertainty", topic, "uncertainty");
        m_walking_twist_uncertainty = header + "/" + topic;

        // Gait state topics
        nh.param<std::string>("topics/gait_state/header", header, "gait_state");
        nh.param<std::string>("topics/" + header + "/gait_state", topic, "gait_state");
        m_gait_state = header + "/" + topic;

        //---------------------------------------------------------------
        // Sensor Topics
        //---------------------------------------------------------------
        // Camera topics
        nh.param<std::string>("topics/camera/header", header, "camera");
        nh.param<std::string>("topics/" + header + "/image_raw", topic, "image_raw");
        m_camera_image_raw = header + "/" + topic;
        nh.param<std::string>("topics/" + header + "/camera_info", topic, "camera_info");
        m_camera_info = header + "/" + topic;
        nh.param<std::string>("topics/" + header + "/target_track", topic, "target_track");
        m_camera_target_track = header + "/" + topic;

        // IMU topics
        nh.param<std::string>("topics/imu/header", header, "imu");
        nh.param<std::string>("topics/" + header + "/data_raw", topic, "data_raw");
        m_imu_data_raw = header + "/" + topic;
        nh.param<std::string>("topics/" + header + "/data_filtered", topic, "data_filtered");
        m_imu_data_filtered = header + "/" + topic;
        nh.param<std::string>("topics/" + header + "/orientation_cov", topic, "orientation_cov");
        m_imu_orientation_cov = header + "/" + topic;

        // Range sensor topics
        nh.param<std::string>("topics/range/header", header, "range");
        nh.param<std::string>("topics/" + header + "/data_raw", topic, "data_raw");
        m_range_data_raw = header + "/" + topic;
        nh.param<std::string>("topics/" + header + "/point_cloud", topic, "point_cloud");
        m_range_point_cloud = header + "/" + topic;
        nh.param<std::string>("topics/" + header + "/filtered", topic, "filtered");
        m_range_filtered = header + "/" + topic;

        // Other sensor related topics to publish
        nh.param<std::string>("topics/temperature/header", header, "temperature");
        nh.param<std::string>("topics/" + header + "/cpu", topic, "cpu");
        m_cpu_temp = header + "/" + topic;

        //---------------------------------------------------------------
        // Odometry Topics
        //---------------------------------------------------------------
        nh.param<std::string>("topics/odometry/header", header, "odometry");
        nh.param<std::string>("topics/" + header + "/ekf", topic, "ekf");
        m_odometry_ekf = header + "/" + topic;

        //---------------------------------------------------------------
        // Target Track Topics
        //---------------------------------------------------------------
        nh.param<std::string>("topics/target_track/header", header, "target_track");
        nh.param<std::string>("topics/" + header + "/image_states", topic, "image_states");
        m_target_image_track = header + "/" + topic;
        nh.param<std::string>("topics/" + header + "/track_mode", topic, "track_mode");
        m_target_track_mode = header + "/" + topic;

    }

    //---------------------------------------------------------------
    // Getters to access class member topic labels
    //---------------------------------------------------------------
    // Leg topics
    std::string leg_foot_states( int indx )
    {
        return m_leg_header + std::to_string(indx) + "/" + m_leg_foot_states;
    }
    std::string leg_joint_states( int indx )
    {
        return m_leg_header + std::to_string(indx) + "/" + m_leg_joint_states;
    }

    // Kinematic topics
    std::string seeker_nominal() { return m_seeker_nominal; }
    std::string seeker_search() { return m_seeker_search; }
    std::string seeker_filtered() { return m_seeker_filtered; }

    std::string body_pose_commanded() { return m_body_pose_commanded; }
    std::string body_pose_gravity_adjusted() { return m_body_pose_gravity_adjusted; }
    std::string body_pose_filtered() { return m_body_pose_filtered; }

    std::string walking_twist_commanded() { return m_walking_twist_commanded; }
    std::string walking_twist_filtered() { return m_walking_twist_filtered; }
    std::string walking_twist_uncertainty() { return m_walking_twist_uncertainty; }

    std::string gait_state() { return m_gait_state; }

    //---------------------------------------------------------------
    // Sensor Topics
    //---------------------------------------------------------------
    // Camera topics
    std::string camera_image_raw() { return m_camera_image_raw; }
    std::string camera_info() { return m_camera_info; }
    std::string camera_target_track() { return m_camera_target_track; }

    // IMU topics
    std::string imu_data_raw() { return m_imu_data_raw; }
    std::string imu_data_filtered() { return m_imu_data_filtered; }
    std::string imu_orientation_cov() { return m_imu_orientation_cov; }

    // Range sensor topics
    std::string range_data_raw() { return m_range_data_raw; }
    std::string range_point_cloud() { return m_range_point_cloud; }
    std::string range_filtered() { return m_range_filtered; }

    // Other sensor topics
    std::string cpu_temp() { return m_cpu_temp; }

    //---------------------------------------------------------------
    // Odometry Topics
    //---------------------------------------------------------------
    std::string odometry_ekf() { return m_odometry_ekf; }

    //---------------------------------------------------------------
    // Target Track Topics
    //---------------------------------------------------------------
    std::string target_image_track() { return m_target_image_track; }
    std::string target_track_mode() { return m_target_track_mode; }

private:
    // =================================================================
    //  Kinematic topics
    // =================================================================
    std::string m_leg_header;
    std::string m_leg_foot_states;
    std::string m_leg_joint_states;

    std::string m_seeker_nominal;
    std::string m_seeker_search;
    std::string m_seeker_filtered;

    std::string m_body_pose_commanded;
    std::string m_body_pose_gravity_adjusted;
    std::string m_body_pose_filtered;

    std::string m_walking_twist_commanded;
    std::string m_walking_twist_filtered;
    std::string m_walking_twist_uncertainty;

    std::string m_gait_state;

    // =================================================================
    //  Sensor topics
    // =================================================================
    // Camera topics
    std::string m_camera_image_raw;
    std::string m_camera_info;
    std::string m_camera_target_track;

    // IMU topics
    std::string m_imu_data_raw;
    std::string m_imu_data_filtered;
    std::string m_imu_orientation_cov;

    // Range sensor topics
    std::string m_range_data_raw;
    std::string m_range_point_cloud;
    std::string m_range_filtered;

    // Other sensor topics
    std::string m_cpu_temp;

    // =================================================================
    //  Odometry topics
    // =================================================================
    std::string m_odometry_ekf;

    // =================================================================
    //  Target Track topics
    // =================================================================
    std::string m_target_image_track;
    std::string m_target_track_mode;
};

#endif
