#ifndef DEPTHCAMERA_H
#define DEPTHCAMERA_H

#include <opencv2/opencv.hpp>

namespace pose
{
class DepthCamera
{
public:
    ~DepthCamera();

    /**
     * @brief Open the device.
     * @return true if the device was successfully opened.
     */
    virtual bool open() = 0;

    /**
     * @brief Close the device.
     */
    virtual void close() = 0;

    /**
     * @brief Wait until the next available frame is processed. This function also attempts to
     * reconstruct the projection matrix on each acquired frame until the reconstruction process
     * succeeds. Projection matrix reconstruction only fails if the image is (mostly) empty.
     */
    void waitForData();

    /**
     * @brief Check whether the device is ready to process. This is true if all
     * images and data is set, esp. if the projection matrix could be reconstructed.
     */
    const bool ready() const;

    /**
     * @brief Get the most recent depth map after calling waitForData().
     */
    const cv::Mat& getDepthMap() const;

    /**
     * @brief Get the most recent point cloud after calling waitForData().
     */
    const cv::Mat& getPointCloud() const;

    /**
     * @brief Get the projection matrix. This is the 3x4 matrix that transforms a given
     * 3D world point to an image point.
     */
    const cv::Mat& getProjectionMatrix() const;

protected:
    DepthCamera();

    virtual void iWaitForData() = 0;

    cv::Mat m_depthMap;
    cv::Mat m_pointCloud;

private:
    cv::Mat computeProjectionMatrix(const cv::Mat& pointCloud) const;

    cv::Mat m_projectionMatrix;
};
}

#endif // DEPTHCAMERA_H
