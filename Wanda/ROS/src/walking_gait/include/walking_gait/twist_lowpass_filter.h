#ifndef __TWIST_LPF_H__
#define __TWIST_LPF_H__

// ROS libraries
#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

// Custom libraries
#include "topic_messages/topics.h"

//=============================================================================
class twist_lowpass_filter
{

public:

    // Constructor
    twist_lowpass_filter( ros::NodeHandle* nh_in );
    void filter_twist();

private:

    // Subscriber callback function
    void get_twist_commanded( const geometry_msgs::TwistStamped& twist );

    float low_pass_filter(
        float state_current,
        float state_old,
        float period
    );

    void process_user_inputs();

    geometry_msgs::Twist init_twist();

    //-------------------------------------------------------
    geometry_msgs::TwistStamped twist_commanded;
    geometry_msgs::TwistStamped twist_filtered;

    std::string subscribed_topic_name;
    std::string published_topic_name;

    const float filter_period;

    ros::NodeHandle nh;

    ros::Publisher pub;
    ros::Subscriber sub;
};

#endif
