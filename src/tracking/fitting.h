#ifndef FITTING_H
#define FITTING_H

#include <opencv2/opencv.hpp>
#include <memory>

namespace pose
{
class Skeleton;
class Joint;
class FittingMethod;

class Fitting
{
public:
    Fitting();
    ~Fitting();

    void process(const cv::Mat& depthMap,
                 const cv::Mat& pointCloud,
                 const cv::Mat& labelMap,
                 const cv::Mat& projectionMatrix);

private:
    void create(const cv::Mat& labelMap);
    void update(const cv::Mat& depthMap, const cv::Mat& labelMap, const cv::Mat& pointCloud, const cv::Mat& projectionMatrix);
    void draw(const cv::Mat& depthMap, const cv::Mat& pointCloud);
    void drawJoint(const std::shared_ptr<Joint>& joint, cv::Mat& dispImg);

    std::map<unsigned int, std::shared_ptr<Skeleton>> m_skeletons;
    std::map<unsigned int, cv::Mat> m_skeletonMasks;

    FittingMethod* m_method;
};
}

#endif // FITTING_H
