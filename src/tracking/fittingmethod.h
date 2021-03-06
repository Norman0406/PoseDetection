#ifndef FITTINGMETHOD_H
#define FITTINGMETHOD_H

#include <memory>

#pragma warning(disable: 4996)
#include <flann/flann.hpp>
#pragma warning(default: 4996)

#include "skeleton.h"
#include <utils/module.h>

namespace pose
{
class Bone;

class FittingMethod
        : public Module
{
public:
    virtual ~FittingMethod();

    void process(const cv::Mat& depthMap,
                 const cv::Mat& pointCloud,
                 const flann::Matrix<float>& flannDataset,
                 std::shared_ptr<Skeleton> skeleton,
                 const cv::Mat& projectionMatrix);

protected:
    FittingMethod();

    virtual void iProcess(const cv::Mat& depthMap,
                          const cv::Mat& pointCloud,
                          std::shared_ptr<Skeleton> skeleton,
                          const cv::Mat& projectionMatrix) = 0;

    float updateSkeleton(std::shared_ptr<Skeleton> skeleton, const cv::Mat& pointCloud, const cv::Mat& projectionMatrix);
    float evaluateJointEnergy(std::shared_ptr<Joint> joint, const cv::Mat& pointCloud, const cv::Mat& projectionMatrix);
    float evaluateBoneEnergy(std::shared_ptr<Bone> bone, const cv::Mat& pointCloud, const cv::Mat& projectionMatrix);

private:
    bool nearestPoint(const cv::Point3f& point, const cv::Mat& pointCloud, const cv::Mat& projectionMatrix, cv::Point3f& nearest, float& distSqr);
    bool nearestPointFlann(const cv::Point3f& point, cv::Point3f& nearest, float& distSqr);
    bool nearestPointUnderneath(const cv::Point3f& point, const cv::Mat& pointCloud, const cv::Mat& projectionMatrix, cv::Point3f& nearest, float& distSqr);
    bool nearestPoint8Conn(const cv::Point3f& point, const cv::Mat& pointCloud, const cv::Mat& projectionMatrix, cv::Point3f& nearest, float& distSqr);

    float m_searchRadius;
    float m_searchRadiusSqr;
    bool m_updateFlannIndex;
    const flann::Matrix<float>* m_flannDataset;
    flann::IndexParams m_flannIndexParams;
    flann::SearchParams m_flannSearchParams;
    flann::Index<flann::L2<float>>* m_flannIndex;
};
}

#endif // FITTINGMETHOD_H
