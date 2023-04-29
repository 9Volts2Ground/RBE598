#ifndef __IMU_ORIENTATION_COV_H
#define __IMU_ORIENTATION_COV_H

// ROS libraries
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

// Custom libraries
#include "topic_messages/topics.h"

//=============================================================================
class imu_orientation_covariance
{

public:

    // Constructor
    imu_orientation_covariance( ros::NodeHandle* nh_in );

private:

    // Subscriber callback function
    void calc_imu_covariance( const sensor_msgs::Imu& imu );

    ros::NodeHandle nh;

    ros::Publisher pub;
    ros::Subscriber sub;
};

#endif
