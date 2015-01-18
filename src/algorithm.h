#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <opencv2/opencv.hpp>

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

private:
    Input* m_input;
    StaticMap* m_staticMap;
    ConnectedComponentLabeling* m_ccLabelling;
    Tracking* m_tracking;
    Fitting* m_fitting;

    int m_width;
    int m_height;
};
}

#endif // ALGORITHM_H
