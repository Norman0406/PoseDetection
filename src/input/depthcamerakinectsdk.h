#ifndef DEPTHCAMERAKINECTSDK_H
#define DEPTHCAMERAKINECTSDK_H

#include "depthcamera.h"
#include <Windows.h>
#include <NuiApi.h>
#include <thread>
#include <mutex>
#include <condition_variable>

class DepthCameraKinectSDK
        : public DepthCamera
{
public:
    DepthCameraKinectSDK();
    ~DepthCameraKinectSDK();

    bool open();
    void close();

    void waitForData();
    const cv::Mat& getDepthMap() const;
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& getPointCloud() const;

private:
    void processDepth();

    INuiSensor* m_sensor;
    HANDLE m_nextDepthFrameEvent;
    HANDLE m_depthStreamHandle;

    std::thread* m_processThread;
    std::mutex m_terminateMutex;
    bool m_terminate;

    std::mutex m_depthMapMutex;
    std::condition_variable m_depthMapReadyCond;
    bool m_depthMapReady;
    cv::Mat m_depthMap;
    cv::Mat m_depthMapBuffer;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_pointCloud;
};

#endif // DEPTHCAMERAKINECTSDK_H
