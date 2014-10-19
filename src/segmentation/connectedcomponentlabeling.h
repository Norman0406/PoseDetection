#ifndef CONNECTEDCOMPONENTLABELING_H
#define CONNECTEDCOMPONENTLABELING_H

#include <opencv2/opencv.hpp>

struct BoundingBox
{
    cv::Point   minPoint;
    cv::Point   maxPoint;
    float       minDepth;
    float       maxDepth;
};

struct ConnectedComponent
{
    int         id;
    cv::Point   centerOfMass;
    float       centerDepth;
    BoundingBox boundingBox;
    int         area;
    std::vector<int>        nearbyIds;
    std::vector<cv::Point>  contour;
};

class ConnectedComponentLabeling
{
public:
    ConnectedComponentLabeling();
    ~ConnectedComponentLabeling();

    void setMinRatio(int minRatio);
    void setMaxDistance(float maxDistance);

    const cv::Mat& getLabelMap() const;
    cv::Mat getColoredLabelMap() const;

    void init();
    void process(const cv::Mat& depthMap);

private:
    bool getNextSeed(const cv::Mat& depthMap, cv::Point& seed);
    bool getNextUnlabelledPoint(const cv::Mat& depthMap, cv::Point& nextPoint);
    bool getClosestUnlabelledPoint(const cv::Mat& depthMap, cv::Point& closestPoint);
    void findConnectedComponents(const cv::Mat& depthMap, const cv::Point& seed, unsigned int label);
    void filterComponents();
    void getRandomColor();

    cv::Mat m_labelMap;
    std::vector<unsigned int> m_labelsToRemove;

    int m_minRatio;
    int m_minSize;
    float m_maxDistance;
};

#endif // CONNECTEDCOMPONENTLABELING_H
