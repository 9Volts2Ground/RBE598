#include <ros/ros.h>
#include <string>

// Custom classes
#include "topic_messages/topics.h"
#include "walking_gait/twist_uncertainty_calc.h"

//===============================================
// Constructor
twist_uncertainty_calc::twist_uncertainty_calc( ros::NodeHandle* nh_in ):
nh( *nh_in ),
uncert_static_lin_x( 0.00005 ),
uncert_static_lin_y( 0.00005 ),
uncert_static_ang_z( 0.00005 ),
uncert_scale_factor_x( 0.1 ),
uncert_scale_factor_y( 0.1 ),
uncert_scale_factor_z( 0.1 )
{

    // ToDo: allow input params to tune the uncertainties
    process_user_inputs();

    // Initialize ROS-stuff
    pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>( topic_out, 1 );
    sub = nh.subscribe( topic_in,
                        10,
                        &twist_uncertainty_calc::calc_twist_uncertainty,
                        this );
}

//===============================================
void twist_uncertainty_calc::calc_twist_uncertainty( const geometry_msgs::TwistStamped& twist )
{
    // Grab the header info of the input twist
    m_twist_u.header = twist.header;
    m_twist_u.twist.twist = twist.twist;

    //-------------------------------------------
    // Set up our covariances
    //-------------------------------------------

    // If we are not moving, set the values super small
    // Otherwise, scale them by how fast we are moving
    if ( fabs( twist.twist.linear.x ) < 0.0005 ){
        uncert_lin_x = uncert_static_lin_x;
    }
    else{
        uncert_lin_x = fabs( twist.twist.linear.x * uncert_scale_factor_x );
    }

    if ( fabs( twist.twist.linear.y ) < 0.0005 ){
        uncert_lin_y = uncert_static_lin_y;
    }
    else{
        uncert_lin_y = fabs( twist.twist.linear.y * uncert_scale_factor_y );
    }

    if ( fabs( twist.twist.angular.z ) < 0.0005 ){
        uncert_ang_z = uncert_static_ang_z;
    }
    else{
        uncert_ang_z = fabs( twist.twist.angular.z * uncert_scale_factor_z );
    }

    // Shove values into the covariance matrix
    m_twist_u.twist.covariance[  0 ] = uncert_lin_x*uncert_lin_x;
    m_twist_u.twist.covariance[  7 ] = uncert_lin_y*uncert_lin_y;
    m_twist_u.twist.covariance[ 35 ] = uncert_ang_z*uncert_ang_z;

    // Set cross-coupling terms
    m_twist_u.twist.covariance[ 1 ] = uncert_lin_x*uncert_lin_y;
    m_twist_u.twist.covariance[ 6 ] = uncert_lin_x*uncert_lin_y;

    m_twist_u.twist.covariance[  5 ] = uncert_lin_x*uncert_ang_z;
    m_twist_u.twist.covariance[ 30 ] = uncert_lin_x*uncert_ang_z;

    m_twist_u.twist.covariance[ 11 ] = uncert_lin_y*uncert_ang_z;
    m_twist_u.twist.covariance[ 31 ] = uncert_lin_y*uncert_ang_z;

    // Send the twist mess out to the world
    pub.publish( m_twist_u );
}

//===============================================
void twist_uncertainty_calc::process_user_inputs()
{
    std::string node_name = ros::this_node::getName();

    // Allow users to change the topic names
    Topics top( nh );
    nh.param<std::string>( node_name + "/topic_in", topic_in, top.walking_twist_filtered() );
    nh.param<std::string>( node_name + "/topic_out", topic_out, top.walking_twist_uncertainty() );

    // Allow users to adjust the uncertainty scale factors
    nh.param<float>( node_name + "/uncert_x", uncert_scale_factor_x, 0.1 );
    nh.param<float>( node_name + "/uncert_y", uncert_scale_factor_y, 0.1 );
    nh.param<float>( node_name + "/uncert_z", uncert_scale_factor_z, 0.1 );
}

//=============================================================================
int main( int argc, char** argv){

    ros::init(argc, argv, "twist_uncertainty_calc_node");
    ros::NodeHandle node_handler;

    twist_uncertainty_calc tuc( &node_handler );
    ros::spin();

    return 0;
}

