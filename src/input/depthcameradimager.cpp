#include "depthcameradimager.h"
#include "Dimagerdll.h"

namespace pose
{
const float DepthCameraDImager::m_sx    = 0.000024f;
const float DepthCameraDImager::m_sy    = 0.000024f;
const int DepthCameraDImager::m_uc      = 80;
const int DepthCameraDImager::m_vc      = 60;
const int DepthCameraDImager::m_k       = -36120;
const float DepthCameraDImager::m_f     = 0.00396f;

DepthCameraDImager::DepthCameraDImager()
    : m_kdat(0), m_ndat(0)
{
    m_depthSize = cv::Size(160, 120);
    m_isInit = false;
}

DepthCameraDImager::~DepthCameraDImager()
{
    close();
}

bool DepthCameraDImager::open()
{
    int ret = InitImageDriver();
    if (ret != 0)
        return false;
    m_isInit = true;

    // go to non-sleep mode
    if (!setSleep(1))
        return false;

    // set framerate
    if (!setFramerate(30))
        return false;

    // set frequency
    if (!setFrequency(0))
        return false;

    // set a default grayscale threshold to filter out noise
    setThreshold(5);

    m_depthMap = cv::Mat(m_depthSize.height, m_depthSize.width, CV_32F);
    m_pointCloud = cv::Mat(m_depthSize.height, m_depthSize.width, CV_32FC3);

    m_kdat = new unsigned short[m_depthSize.width * m_depthSize.height];
    m_ndat = new unsigned short[m_depthSize.width * m_depthSize.height];

    return true;
}

void DepthCameraDImager::close()
{
    if (m_isInit) {
        FreeImageDriver();
        m_isInit = false;

        delete[] m_kdat;
        m_kdat = 0;
        delete[] m_ndat;
        m_ndat = 0;
    }
}

bool DepthCameraDImager::setFrequency(int frq)
{
    if (!m_isInit)
        return false;

    int ret = ChangeFreq(frq);
    if (ret != 0)
        return false;

    if (Freqmode() != frq) {
        ret = ChangeFreq(frq);
        if (ret != 0 || Freqmode() != frq)
            return false;
    }

    return true;
}

bool DepthCameraDImager::getFrequency(int& frq) const
{
    if (!m_isInit)
        return false;

    int ret = Freqmode();
    if (ret == 4)
        return false;

    frq = ret;
    return true;
}

bool DepthCameraDImager::setFramerate(int framerate)
{
    if (!m_isInit)
        return false;

    int ret = ChangeSpeed(framerate);
    if (ret != 0)
        return false;

    if (Speedmode() != framerate) {
        ret = ChangeSpeed(framerate);
        if (ret != 0 || Speedmode() != framerate)
            return false;
    }

    return true;
}

bool DepthCameraDImager::getFramerate(int& speed) const
{
    if (!m_isInit)
        return false;

    int ret = Speedmode();
    if (ret == 99)
        return false;

    speed = ret;
    return true;
}

bool DepthCameraDImager::setSleep(int sleepMode)
{
    if (!m_isInit)
        return false;

    int ret = ChangeSleep(sleepMode);
    if (ret != 0)
        return false;

    if (Sleepmode() != sleepMode) {
        ret = ChangeSleep(sleepMode);
        if (ret != 0 || Sleepmode() != sleepMode)
            return false;
    }

    return true;
}

bool DepthCameraDImager::getSleep(int& sleepMode) const
{
    if (!m_isInit)
        return false;

    int ret = Sleepmode();
    if (ret == 4)
        return false;

    sleepMode = ret;
    return true;
}

void DepthCameraDImager::setThreshold(int threshold)
{
    if (threshold < 0)
        m_grayThreshold = 0;
    else if (threshold > 255)
        m_grayThreshold = 255;
    else
        m_grayThreshold = (unsigned short)threshold;
}

int DepthCameraDImager::getThreshold() const
{
    return m_grayThreshold;
}

cv::Size DepthCameraDImager::getDepthSize() const
{
    return m_depthSize;
}

void DepthCameraDImager::depthTo3D(int u, int v, float D, float& x, float& y, float& z)
{
    float tempU = (m_sx * m_sx) * ((u - m_uc) * (u - m_uc));
    float tempV = (m_sy * m_sy) * ((v - m_vc) * (v - m_vc));

    float xd = (m_sx * (u - m_uc)) / (1 + m_k * (tempU + tempV));
    float yd = (m_sy * (v - m_vc)) / (1 + m_k * (tempU + tempV));
    float d = sqrt(xd * xd + yd * yd + m_f * m_f);

    float Dd = D / d;
    x = xd * Dd;
    y = yd * Dd;
    z = m_f * Dd;
}

void DepthCameraDImager::iWaitForData()
{
    m_depthMap.setTo(0);
    m_pointCloud.setTo(0);

    if (GetImageKN(m_kdat, m_ndat) == 0) {
        const unsigned short* bufferRunDepth = m_kdat;
        const unsigned short* bufferRunGray = m_ndat;
        for (int i = 0; i < m_depthMap.rows; i++) {
            float* depthRow = m_depthMap.ptr<float>(i);
            cv::Vec3f* pointCloudRow = m_pointCloud.ptr<cv::Vec3f>(i);
            for (int j = 0; j < m_depthMap.cols; j++) {
                unsigned short gray = *bufferRunGray;

                if (gray > 5) {
                    // set depth value
                    float depth = *bufferRunDepth / 100.0f;
                    depthRow[j] = depth;

                    // set point in point cloud
                    depthTo3D(j, i, depth, pointCloudRow[j][0], pointCloudRow[j][1], pointCloudRow[j][2]);
                }

                bufferRunDepth++;
                bufferRunGray++;
            }
        }
    }
}
}
