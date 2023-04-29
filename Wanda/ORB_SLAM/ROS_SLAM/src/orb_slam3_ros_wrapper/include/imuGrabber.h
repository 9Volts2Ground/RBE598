#ifndef __IMU_GRABBER__
#define __IMU_GRABBER__

// Standard libraries
#include <mutex>
#include <queue>

// ROS libraries
#include <sensor_msgs/Imu.h>

//=============================================================================
class ImuGrabber
{
public:
    // Constructor
    ImuGrabber(){};

    // Class functions
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

//=============================================================================
void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    mBufMutex.lock();
    imuBuf.push(imu_msg);
    mBufMutex.unlock();

    return;
}

#endif