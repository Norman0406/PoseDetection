#include "fittingmethod.h"
#include "bone.h"
#include <utils/utils.h>
#include <functional>

namespace pose
{
FittingMethod::FittingMethod()
    : Module("FittingMethod"),
      m_flannIndex(0),
      m_updateFlannIndex(true),
      m_flannDataset(0)
{
    m_searchRadius = 0.2f;
    m_searchRadiusSqr = m_searchRadius * m_searchRadius;

    m_flannSearchParams = flann::SearchParams(flann::FLANN_CHECKS_UNLIMITED);
    //m_flannSearchParams = flann::SearchParams(64);
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
    begin();

    m_updateFlannIndex = true;
    m_flannDataset = &flannDataset;

    iProcess(depthMap, pointCloud, skeleton, projectionMatrix);

    end();
}

float FittingMethod::updateSkeleton(std::shared_ptr<Skeleton> skeleton, const cv::Mat& pointCloud, const cv::Mat& projectionMatrix)
{
    // update joint hierarchy
    skeleton->update(projectionMatrix);

    // compute skeleton energy
    float energy = evaluateJointEnergy(skeleton->getRootJoint(), pointCloud, projectionMatrix);
    return energy;
}

float FittingMethod::evaluateJointEnergy(std::shared_ptr<Joint> joint, const cv::Mat& pointCloud, const cv::Mat& projectionMatrix)
{
    cv::Point3f point(0, 0, 0);
    float dist = 0;
    nearestPoint(joint->getPosition3d(), pointCloud, projectionMatrix, point, dist);

    float energy = dist;

    const std::vector<std::shared_ptr<Bone>>& bones = joint->getBones();
    for (size_t i = 0; i < bones.size(); i++) {
        energy += evaluateBoneEnergy(bones[i], pointCloud, projectionMatrix);
    }
    return energy;
}

float FittingMethod::evaluateBoneEnergy(std::shared_ptr<Bone> bone, const cv::Mat &pointCloud, const cv::Mat &projectionMatrix)
{
    return evaluateJointEnergy(bone->getJointEnd(), pointCloud, projectionMatrix);
}

bool FittingMethod::nearestPoint(const cv::Point3f& point, const cv::Mat& pointCloud, const cv::Mat& projectionMatrix, cv::Point3f& nearest, float& distSqr)
{
    // try a simple approximation first, then proceed to the more expensive flann
    if (!nearestPoint8Conn(point, pointCloud, projectionMatrix, nearest, distSqr))
        return nearestPointFlann(point, nearest, distSqr);
    return true;
}

bool FittingMethod::nearestPointUnderneath(const cv::Point3f& point, const cv::Mat& pointCloud, const cv::Mat& projectionMatrix, cv::Point3f& nearest, float& distSqr)
{
    cv::Point2f pointImg = Utils::projectPoint(point, projectionMatrix);

    // range check
    if (pointImg.x < 0 || pointImg.x >= pointCloud.cols ||
        pointImg.y < 0 || pointImg.y >= pointCloud.rows)
        return false;

    nearest = pointCloud.ptr<cv::Point3f>((int)pointImg.y)[(int)pointImg.x];
    distSqr = (point.x - nearest.x) * (point.x - nearest.x) +
            (point.y - nearest.y) * (point.y - nearest.y) +
            (point.z - nearest.z) * (point.z - nearest.z);

    /*if (distSqr > m_searchRadiusSqr)
        distSqr = 0;*/
    return true;
}

bool FittingMethod::nearestPoint8Conn(const cv::Point3f& point, const cv::Mat& pointCloud, const cv::Mat& projectionMatrix, cv::Point3f& nearest, float& distSqr)
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

            if (curPoint.z <= 0)
                continue;

            float dSqr = (point.x - curPoint.x) * (point.x - curPoint.x) +
                    (point.y - curPoint.y) * (point.y - curPoint.y) +
                    (point.z - curPoint.z) * (point.z - curPoint.z);

            if (nearestDist < 0) {
                nearestPoint = curPoint;
                nearestDist = dSqr;
            }
            else if (dSqr < nearestDist) {
                nearestPoint = curPoint;
                nearestDist = dSqr;
            }
        }
    }

    if (nearestDist < 0)
        return false;

    nearest = nearestPoint;
    distSqr = nearestDist;
    return true;
}

bool FittingMethod::nearestPointFlann(const cv::Point3f& point, cv::Point3f& nearest, float& distSqr)
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

    if (numPoints == 0)
        return false;

    float* data = m_flannIndex->getPoint(pointsIdx[0]);
    nearest.x = data[0];
    nearest.y = data[1];
    nearest.z = data[2];
    distSqr = nearestDists[0];

    /*if (distSqr > m_searchRadiusSqr)
        distSqr = 0;*/
    return true;
}
}
