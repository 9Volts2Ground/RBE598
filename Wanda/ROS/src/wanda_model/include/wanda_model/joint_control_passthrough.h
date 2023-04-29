#ifndef __JOINT_CONTROL_PASSTHROUGH_H__
#define __JOINT_CONTROL_PASSTHROUGH_H__

// ROS libraries
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#define NUM_LEGS 6
#define NUM_JOINTS 3
#define NUM_SEEKER_JOINTS 2

//=============================================================================
class joint_control_passthrough
{
public:

    // Constructor
    joint_control_passthrough( ros::NodeHandle* nh_in );

    void send_control_commands();

private:

    // Subscriber callback function
    void get_seeker_states( const sensor_msgs::JointState& seeker_state );
    void get_leg_states( const sensor_msgs::JointState& leg_state );

    //-------------------------------------------------------
    const std::string leg_joints[NUM_JOINTS];
    const std::string seeker_joints[NUM_SEEKER_JOINTS];

    ros::NodeHandle nh;

    ros::Rate loop_rate;

    sensor_msgs::JointState m_leg_states[NUM_LEGS];
    sensor_msgs::JointState m_seeker_states;

    ros::Publisher leg_pub[NUM_JOINTS][NUM_LEGS];
    ros::Subscriber leg_sub[NUM_LEGS];

    ros::Publisher seeker_pub[NUM_SEEKER_JOINTS];
    ros::Subscriber seeker_sub;
};

#endif
