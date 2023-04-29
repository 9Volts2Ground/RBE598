/**
*
* Adapted from ORB-SLAM3: Examples/ROS/src/ros_stereo.cc
*
*/

#include "common.h"
#include "imageGrabber.h"

using namespace std;

//=============================================================================
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Stereo");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    if (argc > 1)
    {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }

    ros::NodeHandle node_handler;
    std::string node_name = ros::this_node::getName();
    image_transport::ImageTransport image_transport(node_handler);

    // Process input parameters
    std::string voc_file, settings_file;
    node_handler.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    node_handler.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    // Make sure we are given the files that ORB_SLAM3 needs
    if (voc_file == "file_not_set" || settings_file == "file_not_set")
    {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");
        ros::shutdown();
        return 1;
    }

    // Handle coordinate frame names
    node_handler.param<std::string>(node_name + "/world_frame_id", world_frame_id, "world");
    node_handler.param<std::string>(node_name + "/cam_frame_id", cam_frame_id, "camera");

    // Some applications may not want the result published as a tf frame. Allow it to be disabled
    node_handler.param<bool>(node_name + "/publish_tf", publish_tf, true );

    // Determine if we want to use pangolin or not
    bool enable_pangolin;
    node_handler.param<bool>(node_name + "/enable_pangolin", enable_pangolin, true);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3::System::STEREO;
    ORB_SLAM3::System SLAM(voc_file, settings_file, sensor_type, enable_pangolin);

    ImageGrabber igb(&SLAM);

    message_filters::Subscriber<sensor_msgs::Image> left_sub(node_handler, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(node_handler, "/camera/right/image_raw", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);

    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    setup_ros_publishers(node_handler);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();
    ros::shutdown();

    return 0;
}
