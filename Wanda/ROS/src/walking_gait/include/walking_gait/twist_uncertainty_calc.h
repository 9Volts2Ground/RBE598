#ifndef __TWIST_UNCERTAINTY_CALC_H__
#define __TWIST_UNCERTAINTY_CALC_H__

// ROS libraries
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

//=============================================================================
class twist_uncertainty_calc
{

public:

    // Constructor
    twist_uncertainty_calc( ros::NodeHandle* nh_in );

private:

    // Subscriber callback function
    void calc_twist_uncertainty( const geometry_msgs::TwistStamped& twist );

    void process_user_inputs();

    std::string topic_in;
    std::string topic_out;

    float uncert_static_lin_x;
    float uncert_static_lin_y;
    float uncert_static_ang_z;

    float uncert_lin_x;
    float uncert_lin_y;
    float uncert_ang_z;

    float uncert_scale_factor_x;
    float uncert_scale_factor_y;
    float uncert_scale_factor_z;

    //-------------------------------------------------------
    geometry_msgs::TwistWithCovarianceStamped m_twist_u;

    ros::NodeHandle nh;

    ros::Publisher pub;
    ros::Subscriber sub;
};

#endif
