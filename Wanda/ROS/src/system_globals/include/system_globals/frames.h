#ifndef __TF_FRAMES_H__
#define __TF_FRAMES_H__

#include <string>
#include <vector>

// ROS libraries
#include <ros/ros.h>

//=============================================================================
class Frames
{

public:
    //---------------------------------------------------------------
    // Constructor
    Frames(
        ros::NodeHandle& nh,
        std::string ns="frames"
    )
    {
        set_frames( nh, ns );
    }

    //---------------------------------------------------------------
    // Sets class member frame labels from global ROS parameters
    void set_frames(
        ros::NodeHandle& nh,
        std::string ns="frames"
    )
    {
        nh.param<std::string>( ns + "/body/body", m_body, "body/body" );
        // Intermediate body frames
        nh.param<std::string>( ns + "/body/commanded", m_body_commanded, "body/commanded" );
        nh.param<std::string>( ns + "/body/filtered", m_body_commanded, "body/filtered" );
        nh.param<std::string>( ns + "/body/gravity_adjust", m_body_gravity_adjust, "body/gravity_adjust" );

        nh.param<std::string>( ns + "/imu", m_imu, "imu" );

        nh.param<std::string>( ns + "/seeker/neck_static", m_neck_static, "seeker/neck_static" );
        nh.param<std::string>( ns + "/seeker/neck", m_neck, "seeker/neck" );
        nh.param<std::string>( ns + "/seeker/seeker", m_seeker, "seeker/seeker" );
        nh.param<std::string>( ns + "/seeker/camera", m_camera, "seeker/camera" );
        nh.param<std::string>( ns + "/seeker/ultrasonic", m_ultrasonic, "seeker/ultrasonic" );

        nh.param<std::vector<std::string>>( ns + "/legs/shoulder", m_shoulder );
        nh.param<std::vector<std::string>>( ns + "/legs/hip", m_hip );
        nh.param<std::vector<std::string>>( ns + "/legs/coxa", m_coxa );
        nh.param<std::vector<std::string>>( ns + "/legs/knee", m_knee );
        nh.param<std::vector<std::string>>( ns + "/legs/femur", m_femur );
        nh.param<std::vector<std::string>>( ns + "/legs/ankle", m_ankle );
        nh.param<std::vector<std::string>>( ns + "/legs/tibia", m_tibia );
        nh.param<std::vector<std::string>>( ns + "/legs/foot", m_foot );

        nh.param<std::string>( ns + "/ground/ground", m_ground, "ground" );
        nh.param<std::vector<std::string>>( ns + "/ground/foot", m_foot_ground );
    }

    //---------------------------------------------------------------
    // Getters to access class member frame labels
    std::string body() { return m_body; }
    std::string imu() { return m_imu; }

    std::string body_commanded() { return m_body_commanded; }
    std::string body_filtered() { return m_body_filtered; }
    std::string body_gravity_adjust() { return m_body_gravity_adjust; }

    std::string neck_static() {return m_neck_static; }
    std::string neck() {return m_neck; }
    std::string seeker() { return m_seeker; }
    std::string camera() { return m_camera; }
    std::string ultrasonic() { return m_ultrasonic; }

    std::string shoulder( int indx ) { return m_shoulder[indx]; }
    std::string hip( int indx ) { return m_hip[indx]; }
    std::string coxa( int indx ) { return m_coxa[indx]; }
    std::string knee( int indx ) { return m_knee[indx]; }
    std::string femur( int indx ) { return m_femur[indx]; }
    std::string ankle( int indx ) { return m_ankle[indx]; }
    std::string tibia( int indx ) { return m_tibia[indx]; }
    std::string foot( int indx ) { return m_foot[indx]; }

    std::string ground() { return m_ground; }
    std::string foot_ground( int indx ) { return m_foot_ground[indx]; }

private:
    std::string m_body;
    std::string m_imu;

    // Intermediate body frames
    std::string m_body_commanded;
    std::string m_body_filtered;
    std::string m_body_gravity_adjust;

    // Seeker-related frames
    std::string m_neck_static;
    std::string m_neck;
    std::string m_seeker;
    std::string m_camera;
    std::string m_ultrasonic;

    // Each leg gets its own transform frames
    std::vector<std::string> m_shoulder;
    std::vector<std::string> m_hip;
    std::vector<std::string> m_coxa;
    std::vector<std::string> m_knee;
    std::vector<std::string> m_femur;
    std::vector<std::string> m_ankle;
    std::vector<std::string> m_tibia;
    std::vector<std::string> m_foot;

    std::string m_ground;
    std::vector<std::string> m_foot_ground;
};

#endif