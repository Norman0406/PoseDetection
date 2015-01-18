#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <opencv2/opencv.hpp>

#pragma warning(disable: 4996)
#include <flann/flann.hpp>
#pragma warning(default: 4996)

namespace pose
{
class Input;
class StaticMap;
class ConnectedComponentLabeling;
class Tracking;
class Fitting;

class Algorithm
{
public:
    Algorithm(int width, int height);
    ~Algorithm();

    bool process(float* depthData, int depthDataSize, float* pointsData, int pointsDataSize);

private:
    void buildFlann(const cv::Mat& pointCloud);
    cv::Point3f nearestPoint(const cv::Point3f& point, const cv::Mat& pointCloud, const cv::Mat& projectionMatrix);
    cv::Point3f nearestPointFlann(const cv::Point3f& point);
    cv::Point3f nearestPointUnderneath(const cv::Point3f& point, const cv::Mat& pointCloud, const cv::Mat& projectionMatrix);
    cv::Point3f nearestPoint8Conn(const cv::Point3f& point, const cv::Mat& pointCloud, const cv::Mat& projectionMatrix);

    Input* m_input;
    StaticMap* m_staticMap;
    ConnectedComponentLabeling* m_ccLabelling;
    Tracking* m_tracking;
    Fitting* m_fitting;

    int m_width;
    int m_height;

    float* m_flannData;
    flann::Matrix<float> m_flannDataset;
    flann::IndexParams m_flannIndexParams;
    flann::SearchParams m_flannSearchParams;
    flann::Index<flann::L2<float>>* m_flannIndex;
};
}

#endif // ALGORITHM_H
