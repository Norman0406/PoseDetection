#ifndef INPUT_H
#define INPUT_H

#include <opencv2/opencv.hpp>

namespace pose
{
class Input
{
public:
    Input(int width, int height);
    ~Input();

    void process(const float* depthData, int depthDataSize, const float* pointsData, int pointsDataSize);

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

private:
    cv::Mat computeProjectionMatrix(const cv::Mat& pointCloud) const;

    cv::Mat m_depthMap;
    cv::Mat m_pointCloud;
    cv::Mat m_projectionMatrix;

    int m_width;
    int m_height;
};
}

#endif // INPUT_H
