#include <ros/ros.h>

// Custom classes
#include "system_globals/frames.h"
#include "walking_gait/twist_lowpass_filter.h"

//===============================================
// Constructor
twist_lowpass_filter::twist_lowpass_filter( ros::NodeHandle* nh_in ):
nh( *nh_in ),
filter_period( 1.0 / 3.0 )
{
    // Initialize my variables
    twist_commanded.twist = init_twist();
    twist_filtered.twist = init_twist();
    // Make sure the output twist has a frame, at least to start with
    // This will be overwritten with the frame in the commanded twist
    // once that starts coming i n
    Frames frames( nh );
    twist_commanded.header.frame_id = frames.ground();
    twist_filtered.header.frame_id = frames.ground();

    process_user_inputs();

    // Initialize ROS-stuff
    pub = nh.advertise<geometry_msgs::TwistStamped>( published_topic_name, 1 );
    sub = nh.subscribe( subscribed_topic_name,
                        10,
                        &twist_lowpass_filter::get_twist_commanded,
                        this );
}

//===============================================
void twist_lowpass_filter::process_user_inputs()
{
    std::string node_name = ros::this_node::getName();

    Topics top( nh );
    nh.param<std::string>( node_name + "/topic_in",
                           subscribed_topic_name,
                           top.walking_twist_commanded() );

    nh.param<std::string>( node_name + "/topic_out",
                           published_topic_name,
                           top.walking_twist_filtered() );
}

//===============================================
void twist_lowpass_filter::get_twist_commanded(
    const geometry_msgs::TwistStamped& twist
)
{
    twist_commanded = twist;
}

//===============================================
void twist_lowpass_filter::filter_twist()
{

    ros::Rate loop_rate( 30 );

    while ( nh.ok() ){

        // Filter each of the filter states
        twist_filtered.twist.linear.x = low_pass_filter( twist_filtered.twist.linear.x,
                                                         twist_commanded.twist.linear.x,
                                                         filter_period );

        twist_filtered.twist.linear.y = low_pass_filter( twist_filtered.twist.linear.y,
                                                         twist_commanded.twist.linear.y,
                                                         filter_period );

        twist_filtered.twist.linear.z = low_pass_filter( twist_filtered.twist.linear.z,
                                                         twist_commanded.twist.linear.z,
                                                         filter_period );

        twist_filtered.twist.angular.x = low_pass_filter( twist_filtered.twist.angular.x,
                                                          twist_commanded.twist.angular.x,
                                                          filter_period );

        twist_filtered.twist.angular.y = low_pass_filter( twist_filtered.twist.angular.y,
                                                          twist_commanded.twist.angular.y,
                                                          filter_period );

        twist_filtered.twist.angular.z = low_pass_filter( twist_filtered.twist.angular.z,
                                                          twist_commanded.twist.angular.z,
                                                          filter_period );

        // Grab header stuff from the twist
        twist_filtered.header.frame_id = twist_commanded.header.frame_id;
        twist_filtered.header.stamp = ros::Time::now();

        // Send out the data
        pub.publish( twist_filtered );

        ros::spinOnce();
        loop_rate.sleep();
    }
}

//===============================================
float twist_lowpass_filter::low_pass_filter(
    float state_filter,
    float state_desired,
    float period
)
{
    float state_new;
    state_new = state_filter + period * ( state_desired - state_filter );
    return state_new;
}

//===============================================
geometry_msgs::Twist twist_lowpass_filter::init_twist()
{
    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    return twist;
}

//=============================================================================
int main( int argc, char** argv){

    ros::init(argc, argv, "twist_lowpass_filter_node");
    ros::NodeHandle node_handler;

    twist_lowpass_filter twist_lowpass_filter( &node_handler );
    twist_lowpass_filter.filter_twist();
    ros::spin();

    return 0;
}

