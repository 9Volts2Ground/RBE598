#ifndef __RANGE_FILTER_H__
#define __RANGE_FILTER_H__

#include <string>
#include <vector>

// ROS libraries
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>

// point cloud headers
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// Custom libraries
#include "topic_messages/topics.h"

//=============================================================================
class range_filter
{
public:
    // Constructor
    range_filter( ros::NodeHandle* nh_in );

private:
    //-------------------------------------------
    // Function definitions
    //-------------------------------------------
    void process_arguments();

    void filter_observable( sensor_msgs::Range range );

    void clear_filter();

    bool valid_measurement( sensor_msgs::Range range );

    void process_good_measurement( sensor_msgs::Range range );

    void send_results();

    bool clutter_present( float range );

    // Extracts a TF transformation
    void grab_transform(
        std::string frame_left,
        std::string frame_right,
        Eigen::Matrix3d& rot,
        Eigen::Vector3d& tran,
        ros::Time time_stamp );

    //-------------------------------------------
    // Member variables
    //-------------------------------------------

    // User-tunable parameters
    int num_required_measurements;
    float beam_width_scale;
    double timeout;
    float measurement_range; // How wide of a measurement we'll accept

    // Variables needed for the filter
    float uncert_width;
    int num_measurements;
    float measurement_sum;
    float measurement_mean;
    ros::Time last_measurement_time;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    std::string sensor_frame;

    // Values used for filtering out ground clutter
    bool prevent_ground_clutter;
    std::string ground_frame;
    Eigen::Matrix3d rot_g2r; // Rotation matrix from world to range sensor frames
    Eigen::Vector3d tran_g2r; // Translation from world to range sensor frames

    std::vector<std::vector<float>> point_pos;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 cloud_out;

    ros::NodeHandle nh;

    ros::Publisher pub;
    ros::Subscriber sub;
};

#endif
