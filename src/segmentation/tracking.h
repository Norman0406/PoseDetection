#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/opencv.hpp>
#include <memory>

namespace pose
{
struct ConnectedComponent;

struct TrackingObject
{
    int id;
    std::shared_ptr<ConnectedComponent> latestComponent;
};

class Tracking
{
public:
    Tracking();
    ~Tracking();

    void setSearchRadius(float);

    const cv::Mat& getLabelMap() const;
    cv::Mat getColoredLabelMap();

    void process(const cv::Mat& depthMap,
                 const cv::Mat& labelMap,
                 const std::vector<std::shared_ptr<ConnectedComponent>>& components,
                 const cv::Mat& projectionMatrix);

private:
    int getNextFreeId() const;

    cv::Mat m_labelMap;
    cv::Mat m_coloredLabelMap;

    std::vector<std::shared_ptr<TrackingObject>> m_trackingObjects;
    float m_searchRadius;
    float m_minBoundingBoxOverlap;
};
}

#endif // LABELING_H
