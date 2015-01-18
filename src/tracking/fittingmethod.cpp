#include "fittingmethod.h"

namespace pose
{
FittingMethod::FittingMethod()
{
}

FittingMethod::~FittingMethod()
{
}

void FittingMethod::process(const cv::Mat& depthMap,
                            const cv::Mat& pointCloud,
                            std::shared_ptr<Skeleton> skeleton,
                            cv::Point3f centerOfMass,
                            const cv::Mat& projectionMatrix)
{
    skeleton->setPosition(centerOfMass);
    skeleton->update(projectionMatrix);

    iProcess(depthMap, pointCloud, skeleton, centerOfMass, projectionMatrix);
}
}
