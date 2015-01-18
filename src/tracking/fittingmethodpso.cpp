#include "fittingmethodpso.h"

namespace pose
{
FittingMethodPSO::FittingMethodPSO()
{
}

FittingMethodPSO::~FittingMethodPSO()
{
}

void FittingMethodPSO::iProcess(const cv::Mat& depthMap,
                                const cv::Mat& pointCloud,
                                std::shared_ptr<Skeleton> skeleton,
                                cv::Point3f centerOfMass,
                                const cv::Mat& projectionMatrix)
{

}
}
