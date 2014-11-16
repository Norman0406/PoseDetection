#ifndef UTILS_H
#define UTILS_H

#include <opencv2/opencv.hpp>

namespace pose {
class Utils
{
public:
    static void getColoredLabelMap(const cv::Mat&, cv::Mat&);
    static cv::Mat getColoredLabelMap(const cv::Mat&);

private:
    Utils();
    ~Utils();
};
}

#endif // LABELING_H
