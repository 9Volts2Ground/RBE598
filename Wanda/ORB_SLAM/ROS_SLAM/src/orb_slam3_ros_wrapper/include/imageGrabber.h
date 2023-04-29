#ifndef __IMAGE_GRABBER__
#define __IMAGE_GRABBER__

// Standard libraries
#include <chrono>
#include <mutex>
#include <queue>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <vector>

// ROS libraries
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// Custom libraries
#include "common.h"
#include "imuGrabber.h"

// ORB-SLAM3-specific libraries. Directory is defined in CMakeLists.txt: ${ORB_SLAM3_DIR}
#include "include/System.h"

//=============================================================================
class ImageGrabber
{
public:
    //-------------------------------------------
    // Constructor without IMU info
    ImageGrabber(
        ORB_SLAM3::System* pSLAM
    ):
        // Initialize class members
        mpSLAM(pSLAM)
    {}

    //-------------------------------------------
    // Constructor with IMU info
    ImageGrabber(
        ORB_SLAM3::System* pSLAM,
        ImuGrabber *pImuGb
    ):
        // Initialize class members
        mpSLAM(pSLAM),
        mpImuGb(pImuGb)
    {}

    //-------------------------------------------
    // Class Functions
    void GrabMono(const sensor_msgs::ImageConstPtr& msg);
    void GrabMonoInertial(const sensor_msgs::ImageConstPtr& msg); // Mono

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,
                    const sensor_msgs::ImageConstPtr& msgRight);
    void GrabStereoInertialLeft(const sensor_msgs::ImageConstPtr& msg); // Stereo left
    void GrabStereoInertialRight(const sensor_msgs::ImageConstPtr& msg); // Stereo right
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,
                  const sensor_msgs::ImageConstPtr& msgD);

    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);

    void SyncWithImuMono();
    void SyncWithImuStereo();

private:
    // Class members
    Sophus::SE3f Twc_current; // Transform from world to camera frames, current time stamp

    queue<sensor_msgs::ImageConstPtr> m_img0Buf; // mono
    queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf; // stereo
    std::mutex mBufMutex; // mono
    std::mutex mBufMutexLeft,mBufMutexRight; // stereo

    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;
};

//=============================================================================
void ImageGrabber::GrabMono(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // ORB-SLAM3 runs in TrackMonocular()
    Sophus::SE3f Tcw = mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec());
    Twc_current = Tcw.inverse();

    ros::Time msg_time = msg->header.stamp;

    // Determine if we have any fresh state info to publish
    process_ros_output(
        Twc_current,
        mpSLAM->GetTrackedMapPoints(),
        world_frame_id,
        cam_frame_id,
        msg_time,
        mpSLAM
        );
}

//=============================================================================
void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Main algorithm runs here
    Sophus::SE3f Tcw = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    Twc_current = Tcw.inverse();

    ros::Time msg_time = cv_ptrLeft->header.stamp;

    // Determine if we have any fresh state info to publish
    process_ros_output(
        Twc_current,
        mpSLAM->GetTrackedMapPoints(),
        world_frame_id,
        cam_frame_id,
        msg_time,
        mpSLAM
        );
}

//=============================================================================
void ImageGrabber::GrabMonoInertial(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutex.lock();
    if (!m_img0Buf.empty())
        m_img0Buf.pop();
    m_img0Buf.push(img_msg);
    mBufMutex.unlock();
}

//=============================================================================
void ImageGrabber::GrabStereoInertialLeft(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutexLeft.lock();
    if (!imgLeftBuf.empty())
        imgLeftBuf.pop();
    imgLeftBuf.push(img_msg);
    mBufMutexLeft.unlock();
}

//=============================================================================
void ImageGrabber::GrabStereoInertialRight(const sensor_msgs::ImageConstPtr &img_msg)
{
    mBufMutexRight.lock();
    if (!imgRightBuf.empty())
        imgRightBuf.pop();
    imgRightBuf.push(img_msg);
    mBufMutexRight.unlock();
}

//=============================================================================
void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // ORB-SLAM3 runs in TrackRGBD()
    Sophus::SE3f Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());
    Twc_current = Tcw.inverse();

    ros::Time msg_time = cv_ptrRGB->header.stamp;

    // Determine if we have any fresh state info to publish
    process_ros_output(
        Twc_current,
        mpSLAM->GetTrackedMapPoints(),
        world_frame_id,
        cam_frame_id,
        msg_time,
        mpSLAM
        );
}

//=============================================================================
// Convters a ROS image topic type to a cv matrix type
//=============================================================================
cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    if(cv_ptr->image.type()==0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cout << "Error type" << std::endl;
        return cv_ptr->image.clone();
    }
}

//=============================================================================
void ImageGrabber::SyncWithImuMono()
{
    while(1)
    {
        if (!m_img0Buf.empty()&&!mpImuGb->imuBuf.empty())
        {
            cv::Mat im;
            double tIm = 0;

            tIm = m_img0Buf.front()->header.stamp.toSec();
            if(tIm>mpImuGb->imuBuf.back()->header.stamp.toSec())
                continue;

            this->mBufMutex.lock();
            im = GetImage(m_img0Buf.front());
            ros::Time msg_time = m_img0Buf.front()->header.stamp;
            m_img0Buf.pop();
            this->mBufMutex.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            mpImuGb->mBufMutex.lock();
            if (!mpImuGb->imuBuf.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec() <= tIm)
                {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();

                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);

                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);

                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));

                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();

            // Main algorithm runs here
            Sophus::SE3f Tcw = mpSLAM->TrackMonocular(im, tIm, vImuMeas);
            Twc_current = Tcw.inverse();

            // Determine if we have any fresh state info to publish
            process_ros_output(
                Twc_current,
                mpSLAM->GetTrackedMapPoints(),
                world_frame_id,
                cam_frame_id,
                msg_time,
                mpSLAM
                );
        }

        std::chrono::milliseconds tSleep(1);
        std::this_thread::sleep_for(tSleep);
    }
}

//=============================================================================
void ImageGrabber::SyncWithImuStereo()
{
    const double maxTimeDiff = 0.01;
    while(1)
    {
        cv::Mat imLeft, imRight;
        double tImLeft = 0, tImRight = 0;
        if (!imgLeftBuf.empty()&&!imgRightBuf.empty()&&!mpImuGb->imuBuf.empty())
        {
            tImLeft = imgLeftBuf.front()->header.stamp.toSec();
            tImRight = imgRightBuf.front()->header.stamp.toSec();

            this->mBufMutexRight.lock();
            while((tImLeft-tImRight)>maxTimeDiff && imgRightBuf.size()>1)
            {
                imgRightBuf.pop();
                tImRight = imgRightBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexRight.unlock();

            this->mBufMutexLeft.lock();
            while((tImRight-tImLeft)>maxTimeDiff && imgLeftBuf.size()>1)
            {
                imgLeftBuf.pop();
                tImLeft = imgLeftBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexLeft.unlock();

            if((tImLeft-tImRight)>maxTimeDiff || (tImRight-tImLeft)>maxTimeDiff)
            {
                // std::cout << "big time difference" << std::endl;
                continue;
            }
            if(tImLeft>mpImuGb->imuBuf.back()->header.stamp.toSec())
                continue;

            this->mBufMutexLeft.lock();
            imLeft = GetImage(imgLeftBuf.front());
            ros::Time msg_time = imgLeftBuf.front()->header.stamp;
            imgLeftBuf.pop();
            this->mBufMutexLeft.unlock();

            this->mBufMutexRight.lock();
            imRight = GetImage(imgRightBuf.front());
            imgRightBuf.pop();
            this->mBufMutexRight.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            mpImuGb->mBufMutex.lock();
            if(!mpImuGb->imuBuf.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tImLeft)
                {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();

                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);

                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);

                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));

                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();

            // ORB-SLAM3 runs in TrackStereo()
            Sophus::SE3f Tcw = mpSLAM->TrackStereo(imLeft,imRight,tImLeft,vImuMeas);
            Twc_current = Tcw.inverse();

            // Determine if we have any fresh state info to publish
            process_ros_output(
                Twc_current,
                mpSLAM->GetTrackedMapPoints(),
                world_frame_id,
                cam_frame_id,
                msg_time,
                mpSLAM
                );

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}

#endif