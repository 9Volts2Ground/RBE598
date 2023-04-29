#include <cmath>
#include <ros/ros.h>

// Custom classes
#include "range_sensor_control/range_point_cloud.h"
#include "system_globals/frames.h"

//===============================================
// Constructor
range_point_cloud::range_point_cloud( ros::NodeHandle* nh_in ):
nh( *nh_in ),
max_measurements_default( 100 ),
tfListener( tfBuffer ),
prevent_ground_clutter( false ),
point_volume( false ),
points_per_observable( 1 )
{
    process_arguments();

    // Initialize the point cloud
    cloud_old.width = 0;
    cloud_old.height = 1;

    rot_w2r = Eigen::Matrix3d::Identity();
    rot_g2r = Eigen::Matrix3d::Identity();

    // Initialize ROS-stuff
    Topics top( nh );
    pub = nh.advertise<sensor_msgs::PointCloud2>( top.range_point_cloud(), 10 );
    sub = nh.subscribe( top.range_data_raw(),
                        10,
                        &range_point_cloud::range_to_point_cloud,
                        this );
}

//===============================================
void range_point_cloud::process_arguments()
{
    std::string node_name = ros::this_node::getName();

    Frames frames( nh );
    nh.param<std::string>( node_name + "/sensor_frame", sensor_frame, frames.ultrasonic() );
    nh.param<std::string>( node_name + "/world_frame", world_frame, "odom" );

    nh.param<int>( node_name + "/max_measurements", max_measurements, max_measurements_default );

    // Determine if we want to publish just one point per measurement, or if we
    // want to build an uncertianty cloud around each point
    nh.param<bool>( node_name + "/point_volume", point_volume, false );

    // If we are pointing the seeker at the ground, prevent us from accepting clutter
    nh.param<bool>( node_name + "/filter_clutter", prevent_ground_clutter, false );
    if ( prevent_ground_clutter ){
        nh.param<std::string>( node_name + "/ground_frame", ground_frame, frames.ground() );
    }

    // Set up a region of detections
    point_pos.push_back( {1.0, 0.0, 0.0} );
    if ( point_volume ){
        points_per_observable = 5;

        // Set the 4 points around the miiddle measurement
        point_pos.push_back( {1.0,  1.0,  0.0} );
        point_pos.push_back( {1.0, -1.0,  0.0} );
        point_pos.push_back( {1.0,  0.0,  1.0} );
        point_pos.push_back( {1.0,  0.0, -1.0} );
    }
}

//===============================================
void range_point_cloud::range_to_point_cloud( sensor_msgs::Range range )
{
    // Only keep it if we got a valid range measurement
    if ( range.range < range.min_range ||
         range.range > range.max_range ){
        return;
    }

    int width = cloud_old.width < max_measurements*points_per_observable ?
                cloud_old.width + points_per_observable : max_measurements*points_per_observable;

    // Calculate width of uncertainty
    float uncert_width = range.range * tan( range.field_of_view / 2.0 );

    // See if we need to reject this point due to clutter
    if ( prevent_ground_clutter ){
        bool clutter_found = clutter_present( range.range, uncert_width );
        if ( clutter_found ){
            // This measurement is probably pointing at the ground. Reject it and move on
            return;
        }
    }

    grab_transform( world_frame, sensor_frame, rot_w2r, tran_w2r, ros::Time(0));

    cloud.width = width;
    cloud.height = 1;
    cloud.points.resize( cloud.width * cloud.height );
    for ( size_t indx=0; indx < cloud.points.size(); indx++ ){
        if ( indx < points_per_observable ){
            // Set the new measurement as the first index
            Eigen::Vector3d rng( point_pos[indx][0]*range.range,
                                 point_pos[indx][1]*uncert_width,
                                 point_pos[indx][2]*uncert_width );

            Eigen::Vector3d rng_world( rot_w2r * rng );
            rng_world = rng_world + tran_w2r;

            cloud.points[indx].x = rng_world[0];
            cloud.points[indx].y = rng_world[1];
            cloud.points[indx].z = rng_world[2];
        }
        else{
            // Push all the other points out
            cloud.points[indx].x = cloud_old.points[indx-points_per_observable].x;
            cloud.points[indx].y = cloud_old.points[indx-points_per_observable].y;
            cloud.points[indx].z = cloud_old.points[indx-points_per_observable].z;
        }
    }

    // Shove the data into a ROS point PointCloude2 message
    pcl::toROSMsg( cloud, cloud_out );

    cloud_out.header.stamp = ros::Time::now();
    cloud_out.header.frame_id = world_frame;

    pub.publish( cloud_out );

    // Save off these points as the old cloud
    cloud_old.width = cloud.width;
    cloud_old.height = cloud.height;
    cloud_old.points.resize( cloud_old.width * cloud_old.height );
    for ( size_t indx=0; indx < cloud.points.size(); indx++ ){
        cloud_old.points[indx].x = cloud.points[indx].x;
        cloud_old.points[indx].y = cloud.points[indx].y;
        cloud_old.points[indx].z = cloud.points[indx].z;
    }

    return;
}

//===============================================
void range_point_cloud::grab_transform(
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

//===============================================
bool range_point_cloud::clutter_present(
    float range,
    float uncert_width
)
{
    bool clutter_found( false );

    grab_transform( ground_frame, sensor_frame, rot_g2r, tran_g2r, ros::Time(0) );

    for ( int indx=0; indx < points_per_observable; indx++ ){

        // First check if this point is ground clutter
        Eigen::Vector3d rng( point_pos[indx][0]*range,
                             point_pos[indx][1]*uncert_width,
                             point_pos[indx][2]*uncert_width );

        // Rotate this point into the ground frame
        Eigen::Vector3d rng_world( rot_g2r * rng );

        // Translate it into the ground frame
        rng_world = rng_world + tran_g2r;

        // Is this point in the ground?
        if ( rng_world[2] <= 0.0 ){
            clutter_found = true;
            return clutter_found;
        }
    }

    // Didn't find any clutter, return
    return clutter_found;
}

//=============================================================================
int main( int argc, char** argv){

    ros::init(argc, argv, "range_point_cloud_node");
    ros::NodeHandle node_handler;

    range_point_cloud rpc( &node_handler );
    ros::spin();

    return 0;
}

