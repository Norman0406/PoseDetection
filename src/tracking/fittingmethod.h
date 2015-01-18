#ifndef FITTINGMETHOD_H
#define FITTINGMETHOD_H

#include <memory>
#include "skeleton.h"

namespace pose
{
class FittingMethod
{
public:
    ~FittingMethod();

    void process(const cv::Mat& depthMap,
                 const cv::Mat& pointCloud,
                 std::shared_ptr<Skeleton> skeleton,
                 cv::Point3f centerOfMass,
                 const cv::Mat& projectionMatrix);

protected:
    FittingMethod();

    virtual void iProcess(const cv::Mat& depthMap,
                          const cv::Mat& pointCloud,
                          std::shared_ptr<Skeleton> skeleton,
                          cv::Point3f centerOfMass,
                          const cv::Mat& projectionMatrix) = 0;
};
}

#endif // FITTINGMETHOD_H
