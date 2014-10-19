#ifndef DEPTHCAMERAKINECTSDK2_H
#define DEPTHCAMERAKINECTSDK2_H

#include "depthcamera.h"
#include <Kinect.h>
#include <Windows.h>
#include <thread>
#include <mutex>
#include <condition_variable>

class DepthCameraKinectSDK2
        : public DepthCamera
{
public:
    DepthCameraKinectSDK2();
    ~DepthCameraKinectSDK2();

    bool open();
    void close();

    float getMinReliableDistance() const;
    float getMaxReliableDistance() const;
    float getFOVHorizontal() const;
    float getFOVVertical() const;
    float getFOVDiagonal() const;
    cv::Size getDepthSize() const;

    void waitForData();
    const cv::Mat& getDepthMap() const;
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& getPointCloud() const;

private:
    void processDepth();

    IKinectSensor* m_sensor;
    IDepthFrameSource* m_depthFrameSource;
    IDepthFrameReader* m_depthFrameReader;
    HANDLE m_depthEventHandle;
    ICoordinateMapper* m_mapper;
    CameraSpacePoint* m_points;

    float m_minReliableDistance;
    float m_maxReliableDistance;
    float m_fovHorizontal;
    float m_fovVertical;
    float m_fovDiagonal;
    cv::Size m_depthSize;

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

#endif // DEPTHCAMERAKINECTSDK2_H
