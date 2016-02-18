#include "algorithm.h"
#include <input/input.h>
#include <segmentation/staticmap.h>
#include <segmentation/connectedcomponentlabeling.h>
#include <segmentation/tracking.h>
#include <tracking/fitting.h>

#include <utils/streamreader.h>

#include <utils/utils.h>

namespace pose
{

// TEMP
const int startFrame = 20;
const int endFrame = -1;
const bool loop = false;
StreamReader depthReader("d:/sequences/scene1/depth.seq", startFrame, endFrame, loop);
StreamReader pointsReader("d:/sequences/scene1/pointcloud.seq", startFrame, endFrame, loop);
StreamReader foregroundReader("d:/sequences/scene1/foreground.seq", startFrame, endFrame, loop);

cv::Mat projectionMatrix;

Algorithm::Algorithm(int width, int height)
    : m_width(width),
      m_height(height)
{
    m_input = new Input(width, height);
    m_staticMap = new StaticMap();
    m_ccLabelling = new ConnectedComponentLabeling();
    m_tracking = new Tracking();
    m_fitting = new Fitting();

    depthReader.setStartFrame(startFrame);
    depthReader.setEndFrame(endFrame);
    depthReader.setLoop(loop);

    pointsReader.setStartFrame(startFrame);
    pointsReader.setEndFrame(endFrame);
    pointsReader.setLoop(loop);

    foregroundReader.setStartFrame(startFrame);
    foregroundReader.setEndFrame(endFrame);
    foregroundReader.setLoop(loop);

    // TEMP
    Utils::loadCvMat("d:/sequences/scene1/projection.cvm", projectionMatrix);
}

Algorithm::~Algorithm()
{
    delete m_input;
    delete m_ccLabelling;
    delete m_staticMap;
    delete m_tracking;
    delete m_fitting;
}

bool Algorithm::process(float* depthData, int depthDataSize, float* pointsData, int pointsDataSize)
{
    UNUSED(depthData);
    UNUSED(depthDataSize);
    UNUSED(pointsData);
    UNUSED(pointsDataSize);

    cv::Mat pointCloud;
    cv::Mat foreground;

    depthReader.read(m_depthMap);
    pointsReader.read(pointCloud);
    foregroundReader.read(foreground);

    // detect connected components
    m_ccLabelling->process(foreground, pointCloud);
    cv::imshow("Labels", m_ccLabelling->getColoredLabelMap());

    const cv::Mat& labelMap = m_ccLabelling->getLabelMap();
    std::vector<std::shared_ptr<ConnectedComponent>> components = m_ccLabelling->getComponents();

    // cluster components and track the users
    m_tracking->process(foreground, labelMap, components, projectionMatrix);
    cv::imshow("Tracking Labels", m_tracking->getColoredLabelMap());

    // fit a skeleton inside each user
    m_fitting->process(foreground, pointCloud, m_tracking->getClusters(), m_tracking->getLabelMap(), projectionMatrix);

    if (cv::waitKey(1) == 27)
        return false;
    return true;
}

/*bool Algorithm::process(float* depthData, int depthDataSize, float* pointsData, int pointsDataSize)
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

    return true;
}*/

bool Algorithm::getImage(PoseImageType type, int* width, int* height, int* size, void** data)
{
    switch (type) {
    case IMAGE_DEPTH:
        {
            *width = m_depthMap.cols;
            *height = m_depthMap.rows;
            *size = m_depthMap.cols * m_depthMap.rows * m_depthMap.elemSize();
            *data = m_depthMap.data;
        }
        break;
    case IMAGE_POINTS:
        break;
    case IMAGE_USERSEGMENTATION:
        break;
#ifdef DEBUG_IMAGES
    case IMAGE_BACKGROUND:
        break;
    case IMAGE_FOREGROUND:
        break;
    case IMAGE_REGIONS:
        break;
#else
    case IMAGE_BACKGROUND:
        // TODO: log error
        break;
    case IMAGE_FOREGROUND:
        // TODO: log error
        break;
    case IMAGE_REGIONS:
        // TODO: log error
        break;
#endif
    default:
        return false;
    }

    return true;
}
}
