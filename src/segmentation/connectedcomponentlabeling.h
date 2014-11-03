#ifndef CONNECTEDCOMPONENTLABELING_H
#define CONNECTEDCOMPONENTLABELING_H

#include <opencv2/opencv.hpp>

struct BoundingBox
{
    // direct information
    cv::Point   minPoint;
    cv::Point   maxPoint;
    float       minDepth;
    float       maxDepth;

    // inferred information
    int width;
    int height;
    cv::Point center;

    void update() {
        width = maxPoint.x - minPoint.x;
        height = maxPoint.y - minPoint.y;
        center = cv::Point(minPoint.x + (width / 2), minPoint.y + (height / 2));
    }
};

struct ConnectedComponent
{
    unsigned int    id;
    cv::Point       centerOfMass;
    float           centerDepth;
    BoundingBox     boundingBox;
    int             area;
    std::vector<unsigned int> nearbyIds;
};

class ConnectedComponentLabeling
{
public:
    ConnectedComponentLabeling();
    ~ConnectedComponentLabeling();

    void setMaxDistance(float maxDistance);

    const std::vector<ConnectedComponent*>& getComponents() const;
    const cv::Mat& getLabelMap() const;
    cv::Mat getColoredLabelMap();

    void process(const cv::Mat& foreground);

private:
    void findConnectedComponents(const cv::Mat& foreground, const cv::Point& seed, unsigned int label);
    std::vector<unsigned int> findNearbyComponents(unsigned int id);

    cv::Mat m_labelMap;
    cv::Mat m_coloredLabelMap;
    cv::Mat m_tempComponent;
    std::vector<ConnectedComponent*> m_components;

    float m_maxDistance;

    cv::Mat temp;
};

#endif // CONNECTEDCOMPONENTLABELING_H
