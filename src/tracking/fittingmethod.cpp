#include "fittingmethod.h"
#include <utils/utils.h>

namespace pose
{
FittingMethod::FittingMethod()
    : m_flannIndex(0),
      m_updateFlannIndex(true),
      m_flannDataset(0)
{
    m_flannSearchParams = flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED);
    m_flannIndexParams = flann::KDTreeSingleIndexParams(15);
}

FittingMethod::~FittingMethod()
{
    delete m_flannIndex;
}

void FittingMethod::process(const cv::Mat& depthMap,
                            const cv::Mat& pointCloud,
                            const flann::Matrix<float>& flannDataset,
                            std::shared_ptr<Skeleton> skeleton,
                            const cv::Mat& projectionMatrix)
{
    m_updateFlannIndex = true;
    m_flannDataset = &flannDataset;

    iProcess(depthMap, pointCloud, skeleton, projectionMatrix);
}

cv::Point3f FittingMethod::nearestPoint(const cv::Point3f& point, const cv::Mat& pointCloud, const cv::Mat& projectionMatrix)
{
    cv::Point3f nearestPoint(0, 0, -1);

    // try a simple approximation first, then proceed to the more expensive flann
    nearestPoint = nearestPoint8Conn(point, pointCloud, projectionMatrix);
    if (nearestPoint.z <= 0)
        nearestPoint = nearestPointFlann(point);

    return nearestPoint;
}

cv::Point3f FittingMethod::nearestPointUnderneath(const cv::Point3f& point, const cv::Mat& pointCloud, const cv::Mat& projectionMatrix)
{
    cv::Point2f pointImg = Utils::projectPoint(point, projectionMatrix);

    // range check
    if (pointImg.x < 0 || pointImg.x >= pointCloud.cols ||
        pointImg.y < 0 || pointImg.y >= pointCloud.rows)
        return cv::Point3f(0, 0, -1);

    return pointCloud.ptr<cv::Point3f>((int)pointImg.y)[(int)pointImg.x];
}

cv::Point3f FittingMethod::nearestPoint8Conn(const cv::Point3f& point, const cv::Mat& pointCloud, const cv::Mat& projectionMatrix)
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

cv::Point3f FittingMethod::nearestPointFlann(const cv::Point3f& point)
{
    // build the index on the first call
    if (m_updateFlannIndex) {
        delete m_flannIndex;
        m_flannIndex = new flann::Index<flann::L2<float>>(*m_flannDataset, m_flannIndexParams);
        m_flannIndex->buildIndex();
        m_updateFlannIndex = false;
    }

    std::vector<float> queryPoint(3);
    queryPoint[0] = (float)point.x;
    queryPoint[1] = (float)point.y;
    queryPoint[2] = (float)point.z;

    const int k = 1;
    std::vector<int> pointsIdx(k);
    std::vector<float> nearestDists(k);
    flann::Matrix<int> indices(&pointsIdx[0], 1, k);
    flann::Matrix<float> dists(&nearestDists[0], 1, k);

    // perform knn search
    int numPoints = m_flannIndex->knnSearch(flann::Matrix<float>(&queryPoint[0], 1, 3), indices, dists, k, m_flannSearchParams);

    cv::Point3f nearestPoint(0, 0, -1);

    if (numPoints > 0) {
        float* data = m_flannIndex->getPoint(pointsIdx[0]);
        nearestPoint.x = data[0];
        nearestPoint.y = data[1];
        nearestPoint.z = data[2];
    }

    return nearestPoint;
}
}
