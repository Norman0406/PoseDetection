#ifndef UTILS_H
#define UTILS_H

#include <opencv2/opencv.hpp>

namespace pose
{
class BoundingBox3D;

#define UNUSED(x) (void)(x)

class Utils
{
public:
    static void getColoredLabelMap(const cv::Mat&, cv::Mat&);
    static cv::Mat getColoredLabelMap(const cv::Mat&);
    static bool loadCvMat(const char* filename, cv::Mat& image);
    static bool saveCvMat(const char* filename, const cv::Mat& image);
    static float distance(const BoundingBox3D& box1, const BoundingBox3D& box2, float searchRadius);

private:
    Utils();
    ~Utils();
};
}

#endif // LABELING_H
