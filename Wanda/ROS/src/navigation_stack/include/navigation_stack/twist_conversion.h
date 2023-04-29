#ifndef __TWIST_CONVERSION_H__
#define __TWIST_CONVERSION_H__

// ROS libraries
#include <ros/ros.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

//=============================================================================
// Class used to convert an un-stamped Twist message to a TwistStamped message
class twist_conversion
{

public:
    // Constructor
    twist_conversion( ros::NodeHandle* nh_in );

private:
    void process_user_inputs();

    // Subscriber callback function
    void convert_twist_type( const geometry_msgs::Twist& twist_in );

    std::string subscribed_topic_name;
    std::string published_topic_name;
    std::string frame_id;

    geometry_msgs::TwistStamped twist_out;

    ros::NodeHandle nh;

    ros::Publisher pub;
    ros::Subscriber sub;
};

#endif
