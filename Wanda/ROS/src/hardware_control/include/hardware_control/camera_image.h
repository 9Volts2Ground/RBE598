#ifndef __CAMERA_IMAGE_H__
#define __CAMERA_IMAGE_H__

#include <string>

// ROS libraries
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>

//=============================================================================
class camera_image
{

public:
    // Constructor
    camera_image( ros::NodeHandle* nh_in );

    void publish_video();

private:
    // Private functions
    void processUserParams();
    void getCameraInfo();

    // Class members
    ros::NodeHandle nh;
    std::string node_name;

    // Flags to control the image
    bool is_wanda;
    bool rotate_image;

    std::string output_namespace;

    // Camera info flags and info
    ros::Publisher pub_cam_info;
    sensor_msgs::CameraInfo caminfo;
    bool publish_camera_info;
    std::string camera_info_namespace;
    std::string camera_info_topic;

    // Image info
    std::string image_topic;
    std::string camera_frame;

    // image_transport::ImageTransport image_transport;
    image_transport::Publisher pub;

    // Set up the camera itself
    int camera_source;
    int video_hz;
    // ros::Rate loop_rate;
};

#endif
