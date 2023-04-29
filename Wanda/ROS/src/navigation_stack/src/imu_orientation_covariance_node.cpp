#include <ros/ros.h>

// Custom classes
#include "navigation_stack/imu_orientation_covariance.h"

//===============================================
// Constructor
imu_orientation_covariance::imu_orientation_covariance( ros::NodeHandle* nh_in ):
nh( *nh_in )
{
    // Initialize ROS-stuff
    Topics top( nh );
    pub = nh.advertise<sensor_msgs::Imu>( top.imu_orientation_cov(), 1 );
    sub = nh.subscribe( top.imu_data_filtered(),
                        10,
                        &imu_orientation_covariance::calc_imu_covariance,
                        this );
}

//===============================================
void imu_orientation_covariance::calc_imu_covariance(
    const sensor_msgs::Imu& imu
)
{
    sensor_msgs::Imu imu_out = imu;

    // For now, set the roll and pitch covariances proportional to the
    // accelerometer X and Y components
    imu_out.orientation_covariance[0] = imu.linear_acceleration_covariance[0] * 
                                        imu.linear_acceleration_covariance[4];

    imu_out.orientation_covariance[4] = imu.linear_acceleration_covariance[0] * 
                                        imu.linear_acceleration_covariance[4];

    // Set the yaw uncertainty equal to the Z component of the angular velocity
    imu_out.orientation_covariance[8] = imu.linear_acceleration_covariance[8];

    // Send out the data
    pub.publish( imu_out );
}

//=============================================================================
int main( int argc, char** argv){

    ros::init(argc, argv, "imu_orientation_covariance_node");
    ros::NodeHandle node_handler;

    imu_orientation_covariance imu_cov( &node_handler );
    ros::spin();

    return 0;
}

