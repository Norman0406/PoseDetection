#ifndef DEPTHCAMERADIMAGER_H
#define DEPTHCAMERADIMAGER_H

#include "depthcamera.h"

namespace pose
{
class DepthCameraDImager
        : public DepthCamera
{
public:
    DepthCameraDImager();
    ~DepthCameraDImager();

    bool open();
    void close();

    bool setFrequency(int frq);
    bool getFrequency(int& frq) const;

    bool setFramerate(int framerate);
    bool getFramerate(int& framerate) const;

    bool setSleep(int sleepMode);
    bool getSleep(int& sleepMode) const;

    void setThreshold(int threshold);
    int getThreshold() const;

    cv::Size getDepthSize() const;

protected:
    void iWaitForData();

private:
    void depthTo3D(int u, int v, float D, float& x, float& y, float& z);

    cv::Size m_depthSize;
    bool m_isInit;
    unsigned short m_grayThreshold;

    unsigned short* m_kdat;
    unsigned short* m_ndat;

    // hardcoded internal camera parameters
    static const float m_sx;
    static const float m_sy;
    static const int m_uc;
    static const int m_vc;
    static const int m_k;
    static const float m_f;
};
}

#endif // DEPTHCAMERADIMAGER_H
