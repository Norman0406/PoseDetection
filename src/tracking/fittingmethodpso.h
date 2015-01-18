#ifndef FITTINGMETHODPSO_H
#define FITTINGMETHODPSO_H

#include <memory>
#include <opencv2/opencv.hpp>

#include "fittingmethod.h"

namespace pose
{

// particle swarm optimization
class FittingMethodPSO
        : public FittingMethod
{
public:
    FittingMethodPSO();
    ~FittingMethodPSO();

protected:
    void iProcess(const cv::Mat& depthMap,
                  const cv::Mat& pointCloud,
                  std::shared_ptr<Skeleton> skeleton,
                  cv::Point3f centerOfMass,
                  const cv::Mat& projectionMatrix);
};
}

#endif // FITTINGMETHODPSO_H
