#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <string>

// Custom classes
#include "topic_messages/topics.h"
#include "wanda_model/joint_control_passthrough.h"

//===============================================
// Constructor
joint_control_passthrough::joint_control_passthrough( ros::NodeHandle* nh_in ):
nh( *nh_in ),
leg_joints( {"hip", "knee", "ankle"} ),
seeker_joints( {"azimuth", "elevation"} ),
loop_rate( 10 )
{

    // Initialize ROS-stuff
    Topics top( nh );

    // Set up the seeker pubs and subs
    m_seeker_states.position.resize( NUM_SEEKER_JOINTS );
    for ( int joint=0; joint < NUM_SEEKER_JOINTS; joint++ ){
        m_seeker_states.position[joint] = 0.0; // Initialize the state we care about
        seeker_pub[joint] = nh.advertise<std_msgs::Float64>( "seeker/controller/" + seeker_joints[joint] + "/command", 1 );
    }
    seeker_sub = nh.subscribe( top.seeker_filtered(),
                               10,
                               &joint_control_passthrough::get_seeker_states,
                               this );

    // Set up the leg pubs and subs
    for ( int leg=0; leg < NUM_LEGS; leg++ ){

        leg_sub[leg] = nh.subscribe(
            top.leg_joint_states( leg ),
            10,
            &joint_control_passthrough::get_leg_states,
            this
            );

        m_leg_states[leg].position.resize( NUM_JOINTS );
        for ( int joint=0; joint < NUM_JOINTS; joint++ ){
            m_leg_states[leg].position[joint] = 0.0; //
            leg_pub[joint][leg] = nh.advertise<std_msgs::Float64>( "leg"+std::to_string(leg)+"/controller/" + leg_joints[joint] + "/command", 1 );
        }
    }
}

//===============================================
void joint_control_passthrough::send_control_commands()
{
    std_msgs::Float64 command;
    while ( nh.ok() ){

        // Send out the leg commands
        for ( int leg=0; leg < NUM_LEGS; leg++ ){
            for ( int joint=0; joint < NUM_JOINTS; joint++ ){
                command.data = m_leg_states[leg].position[joint];
                leg_pub[joint][leg].publish( command );
            }
        }

        // Send out seeker commands
        for ( int joint=0; joint < NUM_SEEKER_JOINTS; joint++){
            command.data = m_seeker_states.position[joint];
            seeker_pub[joint].publish( command );
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

//===============================================
void joint_control_passthrough::get_seeker_states(
    const sensor_msgs::JointState& seeker_state
)
{
    for ( int joint=0; joint < NUM_SEEKER_JOINTS; joint++){
        m_seeker_states.position[joint] = seeker_state.position[joint];
    }
}

//===============================================
void joint_control_passthrough::get_leg_states(
    const sensor_msgs::JointState& leg_state
)
{
    // Janky way to extract the leg number from the topic info
    // ToDo: figure out how to pass 2 arguments in a subscriber
    char c_leg = leg_state.header.frame_id[3];
    int leg = (int)c_leg - '0';

    for ( int joint=0; joint < NUM_JOINTS; joint++ ){
        m_leg_states[leg].position[joint] = leg_state.position[joint];
    }
}

//=============================================================================
int main( int argc, char** argv){

    ros::init(argc, argv, "joint_control_passthrough_node");
    ros::NodeHandle node_handler;

    joint_control_passthrough jcp( &node_handler );

    // Loop through and publish data
    jcp.send_control_commands();

    ros::spin();

    return 0;
}

