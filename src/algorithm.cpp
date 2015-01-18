#include "algorithm.h"
#include <input/input.h>
#include <segmentation/staticmap.h>
#include <segmentation/connectedcomponentlabeling.h>
#include <segmentation/tracking.h>
#include <tracking/fitting.h>

#include <utils/numberedfilereader.h>
#include <utils/utils.h>

namespace pose
{

// TEMP
const int startFrame = 50;
const int endFrame = -1;
const bool loop = true;
NumberedFileReader depthReader("d:/sequences/scene2/depth/depth_%i.cvm", startFrame, endFrame, loop);
NumberedFileReader pointsReader("d:/sequences/scene2/pointcloud/pointcloud_%i.cvm", startFrame, endFrame, loop);
NumberedFileReader foregroundReader("d:/sequences/scene2/foreground/foreground_%i.cvm", startFrame, endFrame, loop);
cv::Mat projectionMatrix;

Algorithm::Algorithm(int width, int height)
    : m_width(width),
      m_height(height),
      m_flannIndex(0),
      m_flannData(0)
{
    m_input = new Input(width, height);
    m_staticMap = new StaticMap();
    m_ccLabelling = new ConnectedComponentLabeling();
    m_tracking = new Tracking();
    m_fitting = new Fitting();

    m_flannSearchParams = flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED);
    m_flannIndexParams = flann::KDTreeSingleIndexParams(15);

    // TEMP
    Utils::loadCvMat("d:/sequences/scene2/projection.cvm", projectionMatrix);
}

Algorithm::~Algorithm()
{
    delete m_input;
    delete m_ccLabelling;
    delete m_staticMap;
    delete m_tracking;
    delete m_fitting;
    delete m_flannIndex;
    delete[] m_flannData;
}

bool Algorithm::process(float* depthData, int depthDataSize, float* pointsData, int pointsDataSize)
{
    UNUSED(depthData);
    UNUSED(depthDataSize);
    UNUSED(pointsData);
    UNUSED(pointsDataSize);

    cv::Mat depthMap;
    cv::Mat pointCloud;
    cv::Mat foreground;

    depthReader.read(depthMap);
    pointsReader.read(pointCloud);
    foregroundReader.read(foreground);

    buildFlann(pointCloud);

    // detect connected components
    m_ccLabelling->process(foreground, pointCloud);
    cv::imshow("Labels", m_ccLabelling->getColoredLabelMap());

    const cv::Mat& labelMap = m_ccLabelling->getLabelMap();
    std::vector<std::shared_ptr<ConnectedComponent>> components = m_ccLabelling->getComponents();

    // cluster components and track the users
    m_tracking->process(depthMap, labelMap, components, projectionMatrix);
    cv::imshow("Tracking Labels", m_tracking->getColoredLabelMap());

    // fit a skeleton inside each user
    m_fitting->process(depthMap, pointCloud, m_tracking->getLabelMap(), projectionMatrix);

    if (cv::waitKey(1) == 27)
        return false;
    return true;
}

void Algorithm::buildFlann(const cv::Mat& pointCloud)
{
    delete[] m_flannData;
    m_flannData = new float[pointCloud.cols * pointCloud.rows * 3];
    memcpy(m_flannData, pointCloud.ptr<float>(), pointCloud.cols * pointCloud.rows * 3 * sizeof(float));
    m_flannDataset = flann::Matrix<float>(m_flannData, pointCloud.cols * pointCloud.rows, 3);
    delete m_flannIndex;
    m_flannIndex = new flann::Index<flann::L2<float>>(m_flannDataset, m_flannIndexParams);
    m_flannIndex->buildIndex();
}

cv::Point3f Algorithm::nearestPoint(const cv::Point3f& point, const cv::Mat& pointCloud, const cv::Mat& projectionMatrix)
{
    cv::Point3f nearestPoint(0, 0, -1);

    // try a simple approximation first, then proceed to the more expensive flann
    nearestPoint = nearestPoint8Conn(point, pointCloud, projectionMatrix);
    if (nearestPoint.z <= 0)
        nearestPoint = nearestPointFlann(point);

    return nearestPoint;
}

cv::Point3f Algorithm::nearestPointUnderneath(const cv::Point3f& point, const cv::Mat& pointCloud, const cv::Mat& projectionMatrix)
{
    cv::Point2f pointImg = Utils::projectPoint(point, projectionMatrix);

    // range check
    if (pointImg.x < 0 || pointImg.x >= pointCloud.cols ||
        pointImg.y < 0 || pointImg.y >= pointCloud.rows)
        return cv::Point3f(0, 0, -1);

    return pointCloud.ptr<cv::Point3f>((int)pointImg.y)[(int)pointImg.x];
}

cv::Point3f Algorithm::nearestPoint8Conn(const cv::Point3f& point, const cv::Mat& pointCloud, const cv::Mat& projectionMatrix)
{
    cv::Point2f pointImg = Utils::projectPoint(point, projectionMatrix);

    cv::Point3f nearestPoint(0, 0, 0);
    float nearestDist = -1;

    // use a simple 8 neighborhood around the projected point to find the nearest neighbor
    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            int iInd = (int)(pointImg.x + i);
            int jInd = (int)(pointImg.y + j);

            if (iInd < 0 || iInd >= pointCloud.cols ||
                jInd < 0 || jInd >= pointCloud.rows)
                continue;

            cv::Point3f curPoint = pointCloud.ptr<cv::Point3f>(jInd)[iInd];

            if (nearestDist < 0) {
                nearestPoint = curPoint;
                nearestDist = (float)cv::norm(curPoint - point);
            }
            else {
                float dist = (float)cv::norm(curPoint - point);
                if (dist < nearestDist) {
                    nearestPoint = curPoint;
                    nearestDist = dist;
                }
            }
        }
    }

    if (nearestDist >= 0)
        return nearestPoint;
    return cv::Point3f(0, 0, -1);
}

cv::Point3f Algorithm::nearestPointFlann(const cv::Point3f& point)
{
    cv::Point3f nearestPoint(0, 0, -1);
    return nearestPoint;
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
}
