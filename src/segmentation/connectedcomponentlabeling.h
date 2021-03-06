#ifndef CONNECTEDCOMPONENTLABELING_H
#define CONNECTEDCOMPONENTLABELING_H

#include <opencv2/opencv.hpp>
#include <memory>
#include <utils/boundingbox2d.h>
#include <utils/boundingbox3d.h>
#include <utils/module.h>

namespace pose
{
struct ConnectedComponent
{
    unsigned int    id;
    cv::Point       centerOfMass;
    float           centerDepth;
    BoundingBox2D   boundingBox2d;
    BoundingBox3D   boundingBox3d;
    int             area;
};

class ConnectedComponentLabeling
        : public Module
{
public:
    ConnectedComponentLabeling();
    ~ConnectedComponentLabeling();

    void setMaxDistance(float maxDistance);

    const std::vector<std::shared_ptr<ConnectedComponent>>& getComponents() const;
    const cv::Mat& getLabelMap() const;
    cv::Mat getColoredLabelMap();

    void process(const cv::Mat& foreground,
                 const cv::Mat& pointCloud);

private:
    void findConnectedComponents(const cv::Mat& foreground,
                                 const cv::Mat& pointcloud,
                                 const cv::Point& seed,
                                 unsigned int label);

    cv::Mat m_labelMap;
    cv::Mat m_coloredLabelMap;
    cv::Mat m_tempMask;
    std::vector<std::shared_ptr<ConnectedComponent> > m_components;

    float m_maxDistance;
};
}

#endif // CONNECTEDCOMPONENTLABELING_H
