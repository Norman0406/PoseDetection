#include "utils.h"

void Utils::getColoredLabelMap(const cv::Mat& labelMap, cv::Mat& coloredLabelMap)
{
    if (coloredLabelMap.empty() || coloredLabelMap.rows != labelMap.rows ||
            coloredLabelMap.cols != labelMap.cols || coloredLabelMap.type() != CV_8UC3) {
        coloredLabelMap = cv::Mat(labelMap.rows, labelMap.cols, CV_8UC3);
    }
    coloredLabelMap.setTo(0);

    for (int i = 0; i < labelMap.cols; i++) {
        for (int j = 0; j < labelMap.rows; j++) {
            const unsigned int& label = labelMap.at<unsigned int>(cv::Point(i, j));

            if (label > 0) {
                srand(label);
                cv::Vec3b col((uchar)((rand() / (float)RAND_MAX) * 255),
                              (uchar)((rand() / (float)RAND_MAX) * 255),
                              (uchar)((rand() / (float)RAND_MAX) * 255));
                coloredLabelMap.at<cv::Vec3b>(cv::Point(i, j)) = col;
            }
        }
    }
}

cv::Mat Utils::getColoredLabelMap(const cv::Mat& labelMap)
{
    cv::Mat coloredLabelMap(labelMap.rows, labelMap.cols, CV_8UC3);
    coloredLabelMap.setTo(0);

    getColoredLabelMap(labelMap, coloredLabelMap);

    return coloredLabelMap;
}
