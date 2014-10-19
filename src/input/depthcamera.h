#ifndef DEPTHCAMERA_H
#define DEPTHCAMERA_H

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class DepthCamera
{
public:
    ~DepthCamera();

    virtual bool open() = 0;
    virtual void close() = 0;

    virtual void waitForData() = 0;
    virtual const cv::Mat& getDepthMap() const = 0;
    virtual const pcl::PointCloud<pcl::PointXYZ>::Ptr& getPointCloud() const = 0;

protected:
    DepthCamera();
};

#endif // DEPTHCAMERA_H
