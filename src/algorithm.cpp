#include "algorithm.h"
#include <input/input.h>
#include <segmentation/staticmap.h>
#include <segmentation/connectedcomponentlabeling.h>
#include <segmentation/tracking.h>
#include <tracking/fitting.h>

namespace pose
{
Algorithm::Algorithm(int width, int height)
    : m_width(width),
      m_height(height)
{
    m_input = new Input(width, height);
    m_staticMap = new StaticMap();
    m_ccLabelling = new ConnectedComponentLabeling();
    m_tracking = new Tracking();
    m_fitting = new Fitting();
}

Algorithm::~Algorithm()
{
    delete m_input;
    delete m_ccLabelling;
    delete m_staticMap;
    delete m_tracking;
    delete m_fitting;
}

void Algorithm::process(float* depthData, int depthDataSize, float* pointsData, int pointsDataSize)
{
    // process input data to create OpenCV images from it and reconstruct the projection matrix
    m_input->process(depthData, depthDataSize, pointsData, pointsDataSize);

    if (m_input->ready()) {
        const cv::Mat& depthMap = m_input->getDepthMap();
        const cv::Mat& pointCloud = m_input->getPointCloud();
        const cv::Mat& projectionMatrix = m_input->getProjectionMatrix();

        // process the depth data and compute a static background
        m_staticMap->process(depthMap);

        const cv::Mat& foreground = m_staticMap->getForeground();

        // detect connected components
        m_ccLabelling->process(foreground, pointCloud);

        const cv::Mat& labelMap = m_ccLabelling->getLabelMap();
        std::vector<std::shared_ptr<ConnectedComponent>> components = m_ccLabelling->getComponents();

        // cluster components and track the users
        m_tracking->process(depthMap, labelMap, components, projectionMatrix);

        // fit a skeleton inside each user
        m_fitting->process(depthMap, pointCloud, labelMap, projectionMatrix);
    }
}
}
