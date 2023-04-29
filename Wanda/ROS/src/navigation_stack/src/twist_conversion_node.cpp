#include <ros/ros.h>

// Custom classes
#include "navigation_stack/twist_conversion.h"
#include "system_globals/frames.h"
#include "topic_messages/topics.h"

//===============================================
// Constructor
twist_conversion::twist_conversion( ros::NodeHandle* nh_in ):
nh( *nh_in )
{
    process_user_inputs();

    // Initialize ROS-stuff
    pub = nh.advertise<geometry_msgs::TwistStamped>( published_topic_name, 1 );
    sub = nh.subscribe( subscribed_topic_name,
                        10,
                        &twist_conversion::convert_twist_type,
                        this );
}

//===============================================
void twist_conversion::process_user_inputs()
{
    std::string node_name = ros::this_node::getName();

    Topics top( nh );
    nh.param<std::string>( node_name + "/topic_in",
                           subscribed_topic_name,
                           "cmd_vel" );

    nh.param<std::string>( node_name + "/topic_out",
                           published_topic_name,
                           top.walking_twist_commanded() );

    Frames frame( nh );
    nh.param<std::string>( node_name + "/frame",
                           frame_id,
                           frame.ground() );
}

//===============================================
void twist_conversion::convert_twist_type( const geometry_msgs::Twist& twist_in )
{
    twist_out.twist = twist_in;
    twist_out.header.frame_id = frame_id;
    twist_out.header.stamp = ros::Time::now();

    pub.publish( twist_out );
}


//=============================================================================
int main( int argc, char** argv){

    ros::init(argc, argv, "twist_conversion_node");
    ros::NodeHandle node_handler;

    twist_conversion tcn( &node_handler );
    ros::spin();

    return 0;
}

