#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <opencv2/opencv.hpp>
#include "pose.h"

namespace pose
{
class Input;
class StaticMap;
class ConnectedComponentLabeling;
class Tracking;
class Fitting;

class Algorithm
{
public:
    Algorithm(int width, int height);
    ~Algorithm();

    bool process(float* depthData, int depthDataSize, float* pointsData, int pointsDataSize);

    bool getImage(PoseImageType type, int* width, int* height, int* size, void** data);

private:
    Input* m_input;
    StaticMap* m_staticMap;
    ConnectedComponentLabeling* m_ccLabelling;
    Tracking* m_tracking;
    Fitting* m_fitting;

    int m_width;
    int m_height;

    cv::Mat m_depthMap;
};
}

#endif // ALGORITHM_H
