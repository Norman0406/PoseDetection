#ifndef UTILS_H
#define UTILS_H

#include <opencv2/opencv.hpp>

namespace pose
{
class Utils
{
public:
    static void getColoredLabelMap(const cv::Mat&, cv::Mat&);
    static cv::Mat getColoredLabelMap(const cv::Mat&);
    static bool loadCvMat(const char* filename, cv::Mat& image);
    static bool saveCvMat(const char* filename, const cv::Mat& image);

private:
    Utils();
    ~Utils();
};
}

#endif // LABELING_H
