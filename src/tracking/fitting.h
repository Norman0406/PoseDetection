#ifndef FITTING_H
#define FITTING_H

#include <opencv2/opencv.hpp>
#include <memory>

#pragma warning(disable: 4996)
#include <flann/flann.hpp>
#pragma warning(default: 4996)

#include <utils/module.h>

namespace pose
{
class Skeleton;
class Joint;
class FittingMethod;
struct TrackingCluster;

class Fitting
        : public Module
{
public:
    Fitting();
    ~Fitting();

    void process(const cv::Mat& foreground,
                 const cv::Mat& pointCloud,
                 const std::vector<std::shared_ptr<TrackingCluster>>& clusters,
                 const cv::Mat& labelMap,
                 const cv::Mat& projectionMatrix);

private:
    void create(const std::vector<std::shared_ptr<TrackingCluster>>& clusters);
    void update(const cv::Mat& foreground, const cv::Mat& labelMap, const cv::Mat& pointCloud, const cv::Mat& projectionMatrix);
    void draw(const cv::Mat& foreground, const cv::Mat& pointCloud);
    void drawJoint(const std::shared_ptr<Joint>& joint, cv::Mat& dispImg);

    std::map<unsigned int, std::shared_ptr<Skeleton>> m_skeletons;
    std::map<unsigned int, cv::Mat> m_skeletonMasks;

    FittingMethod* m_method;

    float* m_flannData;
    flann::Matrix<float> m_flannDataset;
};
}

#endif // FITTING_H
