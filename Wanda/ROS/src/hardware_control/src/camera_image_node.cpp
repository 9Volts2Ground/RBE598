
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "opencv2/opencv.hpp"
#include <sensor_msgs/CameraInfo.h>
#include <string>
#include <vector>

// Custom classes
#include "hardware_control/camera_image.h"
#include "system_globals/frames.h"
#include "topic_messages/topics.h"

// SAS: for debugging
#include <iostream>

//===============================================
// Constructor
camera_image::camera_image( ros::NodeHandle* nh_in ):
nh( *nh_in ),
node_name( ros::this_node::getName() )
{
    processUserParams();

    image_transport::ImageTransport image_transport( nh );
    pub = image_transport.advertise( image_topic, 1 );
}

//=============================================================================
void camera_image::processUserParams()
{
    // Figure out if we want to flip the image over, if the camera is mounted upside down
    nh.param<bool>( "IS_WANDA", is_wanda, false );
    nh.param<bool>( node_name + "/rotate_image", rotate_image, false );

    // Grab namespace to publish everything out to
    nh.param<std::string>( node_name + "/namespace", output_namespace, "camera" );

    // Process if users want to rename the output topic
    if ( !nh.param<std::string>( node_name + "/image_topic", image_topic, "image_raw" ) ){
        // Use the topics class to grab global class names
        Topics top( nh );
        image_topic = top.camera_image_raw();
    }
    else{
        // Prepend the desired namespace to the topic
        image_topic = output_namespace + "/" + image_topic;
    }

    // Allow users to set the TF frame of the camera
    if ( !nh.param<std::string>( node_name + "/camera_frame", camera_frame, "seeker/camera" ) ){
        // Optionally, we can look for frame parameters under the "frames" namespace
        Frames frames( nh );
        camera_frame = frames.camera();
        nh.param<std::string>(  "frames/camera", camera_frame, "seeker/camera" );
    }

    // Allow users to set the frame rate
    nh.param<int>( node_name + "/frame_rate", video_hz, 10 );

    // Allow users to choose which camera hardware to use
    nh.param<int>( node_name + "/camera_source", camera_source, 0 );

    // We can optionally publish CameraInfo along with the image
    nh.param<bool>( node_name + "/publish_camera_info", publish_camera_info, false );
    if ( publish_camera_info ){
        // Grab the camera info parameters
        getCameraInfo();
    }
}

//=============================================================================
void camera_image::getCameraInfo()
{
    // Grab the namespace that all the camera parameters are stored under
    nh.param<std::string>(node_name + "/camera_info_namespace", camera_info_namespace, "camera");

    // Grab image shape
    int width, height;
    nh.param<int>(camera_info_namespace + "/image_width", width, 1280 );
    nh.param<int>(camera_info_namespace + "/image_height", height, 720 );
    caminfo.width = width;
    caminfo.height = height;

    // Grab camera matrix
    std::vector<float> camera_matrix;
    if ( !nh.getParam(camera_info_namespace + "/camera_matrix/data", camera_matrix) ) {
        ROS_WARN( "Could not find camera_matrix/data" );
    }
    else {
        for (unsigned int indx=0; indx < camera_matrix.size(); indx++){
            caminfo.K[indx] = camera_matrix[indx];
        }
    }

    // Grab distortion model type
    std::string distortion_model;
    nh.param<std::string>(camera_info_namespace + "/distortion_model", distortion_model, "plumb_bob");
    caminfo.distortion_model = distortion_model;

    // Grab distortion coeffiicents
    std::vector<float> distortion_coefficients;
    if ( !nh.getParam(camera_info_namespace + "/distortion_coefficients/data", distortion_coefficients) ) {
        ROS_WARN( "Could not find distortion_coefficients/data");
    }
    else {
        for ( unsigned int indx=0; indx < distortion_coefficients.size(); indx++){
            caminfo.P[indx] = distortion_coefficients[indx];
        }
    }

    // Grab rectification matrix
    std::vector<float> rectification_matrix;
    if ( !nh.getParam(camera_info_namespace + "/rectification_matrix/data", rectification_matrix) ) {
        ROS_WARN( "Could not find rectification_matrix/data" );
    }
    else {
        for ( unsigned int indx=0; indx < rectification_matrix.size(); indx++){
            caminfo.R[indx] = rectification_matrix[indx];
        }
    }

    // Grab projection matrix
    std::vector<float> projection_matrix;
    if ( !nh.getParam(camera_info_namespace + "/projection_matrix/data", projection_matrix) ) {
        ROS_WARN( "Could not find projection_matrix/data" );
    }
    else {
        for ( unsigned int indx=0; indx < projection_matrix.size(); indx++){
            caminfo.P[indx] = projection_matrix[indx];
        }
    }

    // Grab the topic name to publish cameraInfo to, under the output_namespace
    if ( !nh.param<std::string>(node_name + "/camera_info_topic", camera_info_topic, "camera_info") ){
        // Use default Topics class
        Topics top( nh );
        camera_info_topic = top.camera_info();
    }
    else{
        // Prepend the desired namespace
        camera_info_topic = output_namespace + "/" + camera_info_topic;
    }

    // Set up the publisher for this data
    pub_cam_info = nh.advertise<sensor_msgs::CameraInfo>(camera_info_topic, 1);
}

//=============================================================================
void camera_image::publish_video(){

    // Set up the camera
    cv::VideoCapture cam( camera_source );

    // Make sure we actually opened the camera.
    // If the camera doesn't work, try a few more times
    int fail_count = 0;
    while (!cam.isOpened() ){
        fail_count++;
        if ( fail_count > 10 ){
            ROS_ERROR( "Could not open video camera source %d. Quitting %s", camera_source, node_name.c_str() );
            return;
        }
    }

    // Set up loop rate
    ros::Rate loop_rate( video_hz );

    // Declare the image from the camera
    cv::Mat cv_image;

    // Declare the image message
    sensor_msgs::ImagePtr msg;

    // Loop forever to capture video feed
    while ( nh.ok() ){

        // Grab image from the camera
        cam >> cv_image;

        // If we actually got video feed, send it out
        if ( !cv_image.empty() ){

            // If we need to spin the image over, do it now
            if ( is_wanda || rotate_image ){
                cv::rotate( cv_image, cv_image, cv::ROTATE_180 );
            }

            // Package the image up into a message using cv_bridge
            msg = cv_bridge::CvImage(
                std_msgs::Header(),
                "bgr8",
                cv_image).toImageMsg();

            msg->header.stamp = ros::Time::now();
            msg->header.frame_id = camera_frame;

            // Publish the image
            pub.publish( msg );

            // Send out cameraInfo along with the image
            if ( publish_camera_info ){
                caminfo.header.stamp == ros::Time::now();
                pub_cam_info.publish( caminfo );
            }
        }

        // Wait until it's time to grab the next frame
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Stop all threads
    cam.release();
    ros::shutdown();
}

//=============================================================================
int main( int argc, char** argv){

    ros::init(argc, argv, "camera_image_node");
    ros::NodeHandle node_handler;

    camera_image cam_node( &node_handler );
    cam_node.publish_video();
    ros::spin();

    return 0;
}
