#include <cmath>
#include <ros/ros.h>

// Custom classes
#include "range_sensor_control/range_filter.h"
#include "system_globals/frames.h"

//===============================================
// Constructor
range_filter::range_filter( ros::NodeHandle* nh_in ):
nh( *nh_in ),
tfListener( tfBuffer )
{
    process_arguments();

    clear_filter();

    rot_g2r = Eigen::Matrix3d::Identity();

    cloud.width = 5;
    cloud.height = 1;
    cloud.points.resize( cloud.width * cloud.height );

    // Set up vectors to define the region of the seeker cone angle
    point_pos.push_back( {1.0,  0.0,  0.0} );
    point_pos.push_back( {1.0,  1.0,  0.0} );
    point_pos.push_back( {1.0, -1.0,  0.0} );
    point_pos.push_back( {1.0,  0.0,  1.0} );
    point_pos.push_back( {1.0,  0.0, -1.0} );

    // Initialize ROS-stuff
    Topics top( nh );
    pub = nh.advertise<sensor_msgs::PointCloud2>( top.range_filtered(), 10 );
    sub = nh.subscribe( top.range_data_raw(),
                        10,
                        &range_filter::filter_observable,
                        this );
}

//===============================================
void range_filter::process_arguments()
{
    std::string node_name = ros::this_node::getName();

    Frames frames( nh );
    nh.param<std::string>( node_name + "/sensor_frame", sensor_frame, frames.ultrasonic() );

    // If we are pointing the seeker at the ground, prevent us from accepting clutter
    nh.param<bool>( node_name + "/filter_clutter", prevent_ground_clutter, false );
    if ( prevent_ground_clutter ){
        nh.param<std::string>( node_name + "/ground_frame", ground_frame, frames.ground() );
    }

    // How many measurements we need to accept it
    nh.param<int>( node_name + "/num_measurements", num_required_measurements, 5 );

    // Allow users to scale the beam width found in the topic
    nh.param<float>( node_name + "/beam_width_scale", beam_width_scale, 1.0 );

    // Allow users to set max time between valid measurements
    nh.param<double>( node_name + "/timeout", timeout, 0.5 );

    // Allow users to set how closely the measurements need to be to each other
    nh.param<float>( node_name + "/measurement_range", measurement_range, 0.05 );
}

//===============================================
void range_filter::filter_observable(
    sensor_msgs::Range range
)
{
    //-------------------------------------------
    // Go through steps to reject this observable
    //-------------------------------------------
    bool valid = valid_measurement( range );
    if ( !valid ){
        return;
    }

    //-------------------------------------------
    // Good measurement. Process it
    //-------------------------------------------
    process_good_measurement( range );

    if ( num_measurements == num_required_measurements ){
        // We have enough detections. Send out the filtered info
        send_results();
    }

    return;
}

//===============================================
void range_filter::clear_filter()
{
    num_measurements = 0;
    measurement_sum = 0.0;
    measurement_mean = 0.0;
    last_measurement_time = ros::Time::now();
}

//===============================================
bool range_filter::valid_measurement(
    sensor_msgs::Range range
)
{
    bool valid( true );

    //-------------------------------------------
    // Go through steps to reject this observable
    //-------------------------------------------
    // This measurement isn't valid. Reject it
    if ( range.range < range.min_range || range.range > range.max_range ){
        valid = false;
        return valid;
    }

    // Make sure it hasn't been too long since we got our first measurement
    if ( ros::Time::now().toSec() - last_measurement_time.toSec() > timeout ){
        // Throw out the old data and start with this fresh measurement
        clear_filter();
    }

    // Determine if this is a clutter return
    uncert_width = range.range * tan( beam_width_scale * range.field_of_view / 2.0 );
    if ( prevent_ground_clutter ){
        bool clutter_found = clutter_present( range.range );
        if ( clutter_found ){
            // This measurement is probably pointing at the ground. Reject it and move on
            valid = false;
            return valid;
        }
    }

    // Make sure the new measurement agrees with the previous measurements
    if ( num_measurements > 0 ){
        // Check that the variance isn't too big
        if ( std::abs( range.range - measurement_mean ) > measurement_range ){
            // This measurement deviates from the previous measurements.
            // Reset the filter and start over with this one
            clear_filter();
            return valid;
        }
    }

    // Haven't found a reason to reject this observation. Keep it
    return valid;
}

//===============================================
void range_filter::process_good_measurement(
    sensor_msgs::Range range
)
{

    last_measurement_time = ros::Time::now();

    num_measurements++;

    measurement_sum += range.range;
    measurement_mean = measurement_sum / num_measurements;
}

//===============================================
void range_filter::send_results()
{
    // Calculate a point cloud with the beam width for this measurement
    for ( size_t indx=0; indx < cloud.points.size(); indx++ ){
        cloud.points[indx].x = point_pos[indx][0]*measurement_mean;
        cloud.points[indx].y = point_pos[indx][1]*uncert_width;
        cloud.points[indx].z = point_pos[indx][2]*uncert_width;
    }

    // Shove the data into a ROS point PointCloude2 message
    pcl::toROSMsg( cloud, cloud_out );

    cloud_out.header.stamp = ros::Time::now();
    cloud_out.header.frame_id = sensor_frame;

    // Publish it out
    pub.publish( cloud_out );

    // Clear out the filter, ready for more measurements
    clear_filter();
}

//===============================================
bool range_filter::clutter_present(
    float range
)
{
    bool clutter_found( false );

    grab_transform( ground_frame, sensor_frame, rot_g2r, tran_g2r, ros::Time(0) );

    // First check if this point is ground clutter
    Eigen::Vector3d rng( range, 0.0, uncert_width );

    // Rotate this point into the ground frame
    Eigen::Vector3d rng_world( rot_g2r * rng );

    // Translate it into the ground frame
    rng_world = rng_world + tran_g2r;

    // Is this point in the ground?
    if ( rng_world[2] <= 0.0 ){
        clutter_found = true;
        return clutter_found;
    }

    // Didn't find any clutter, return
    return clutter_found;
}

//===============================================
void range_filter::grab_transform(
    std::string frame_left,
    std::string frame_right,
    Eigen::Matrix3d& rot,
    Eigen::Vector3d& tran,
    ros::Time time_stamp
)
{
    try{
        geometry_msgs::TransformStamped trans; // Temp transform variable
        trans = tfBuffer.lookupTransform( frame_left, frame_right, time_stamp );
        tran << trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z;

        // Get a rotation matrix from the transform
        Eigen::Quaterniond quat;
        quat = Eigen::Quaterniond(trans.transform.rotation.w,
                                  trans.transform.rotation.x,
                                  trans.transform.rotation.y,
                                  trans.transform.rotation.z);
        rot = Eigen::Matrix3d( quat );
    }
    catch (tf2::TransformException &ex){
        ROS_WARN("%s",ex.what());
    }
}

//=============================================================================
int main( int argc, char** argv){

    ros::init(argc, argv, "range_filter_node");
    ros::NodeHandle node_handler;

    range_filter rpc( &node_handler );
    ros::spin();

    return 0;
}

