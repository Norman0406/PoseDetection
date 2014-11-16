#include "staticmap.h"

namespace pose
{
StaticMap::StaticMap()
{
    m_updateFrames = 0;
    setBackgroundResetRatio(0.2f);
    setBackgroundLockedRatio(0.05f);
    setUpdateDelayFrames(10);
    setForegroundDistance(0.1f);
    setMinRatio(320);
}

StaticMap::~StaticMap()
{
}

void StaticMap::setUpdateDelayFrames(int frames)
{
    m_updateDelayFrames = frames;
}

void StaticMap::setBackgroundResetRatio(float ratio)
{
    m_backgroundResetRatio = ratio;
}

void StaticMap::setBackgroundLockedRatio(float ratio)
{
    m_backgroundLockedRatio = ratio;
}

void StaticMap::setForegroundDistance(float distance)
{
    m_foregroundDistance = distance;
}

void StaticMap::setMinRatio(int minRatio)
{
    m_minRatio = minRatio;
}

void StaticMap::reset()
{
    m_background.setTo(0);
}

const cv::Mat& StaticMap::getBackground() const
{
    return m_background;
}

const cv::Mat& StaticMap::getForeground() const
{
    return m_foreground;
}

void StaticMap::process(const cv::Mat& depthMap)
{
    if (depthMap.cols != m_background.cols || depthMap.rows != m_background.rows) {
        m_foreground = cv::Mat(depthMap.rows, depthMap.cols, CV_32F);
        m_foregroundMask = cv::Mat(depthMap.rows, depthMap.cols, CV_8U);
        m_count = cv::Mat(depthMap.rows, depthMap.cols, CV_32F);
        m_tempContour = cv::Mat(depthMap.rows, depthMap.cols, CV_8U);
        m_minSize = depthMap.cols * depthMap.rows / m_minRatio;
        m_count.setTo(0);

        // create an initial background
        depthMap.copyTo(m_background);
    }
    m_foreground.setTo(0);
    m_foregroundMask.setTo(0);

    // update the background model
    for (int i = 0; i < depthMap.cols; i++) {
        for (int j = 0; j < depthMap.rows; j++) {
            const cv::Point point(i, j);
            const float& dist = depthMap.at<float>(point);
            float& background = m_background.at<float>(point);
            unsigned int& count = m_count.at<unsigned int>(point);

            // update background model with running average
            if (dist > 0 && dist > background - m_foregroundDistance) {
                // cumulative moving average
                background = background + ((dist - background) / (count + 1));

                count++;
            }

            // create foreground mask
            if (dist > 0 && dist < background - m_foregroundDistance) {
                float& foreground = m_foreground.at<float>(point);
                uchar& foregroundMask = m_foregroundMask.at<uchar>(point);
                foreground = dist;
                foregroundMask = 255;
            }
        }
    }

    // filter contours, i.e. filter noise and only take the strongest contours
    filterContours();

    // everything that is not taken as foreground object is added back to the background
    // NOTE: this step balances the noise and stabilizes the background model
    for (int i = 0; i < m_foreground.cols; i++) {
        for (int j = 0; j < m_foreground.rows; j++) {
            cv::Point point(i, j);
            const float& dist = depthMap.at<float>(point);
            const float& foreground = m_foreground.at<float>(point);
            float& background = m_background.at<float>(point);
            unsigned int& count = m_count.at<unsigned int>(point);

            // add value to the background
            if (foreground == 0 && dist != 0)
                background = background + ((dist - background) / (count));
        }
    }

    // percentage of points that have been updated in the background model
    /*float pointsChangedRatio = pointsChanged / (float)totalNumPoints;

    // lock the background if the number of changed points falls below a certain ratio
    if (!m_backgroundLocked && m_updateFrames == 0 && pointsChangedRatio < m_backgroundLockedRatio) {
        m_backgroundLocked = true;
    }
    if (m_backgroundLocked && pointsChangedRatio > m_backgroundResetRatio) {
        reset();
        m_updateFrames = m_updateDelayFrames;
        m_backgroundLocked = false;
    }
    else {
        if (m_updateFrames > 0)
            m_updateFrames--;
    }*/
}

void StaticMap::filterContours()
{
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

    // perform an opening to supress noise
    cv::erode(m_foregroundMask, m_foregroundMask, element);
    cv::dilate(m_foregroundMask, m_foregroundMask, element);

    // find contours in the noise-reduced mask
    m_contours.clear();
    cv::findContours(m_foregroundMask, m_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

    // create a binary mask that contains only contours that are big enough
    m_tempContour.setTo(1);
    #pragma omp parallel for
    for (int i = 0; i < (int)m_contours.size(); i++) {
        // compute the number of contour pixels
        double area = cv::contourArea(m_contours[i]);

        // only draw contours that are big enough
        if (area > m_minSize)
            cv::drawContours(m_tempContour, m_contours, i, cv::Scalar::all(0), CV_FILLED);
    }

    m_foreground.setTo(0, m_tempContour);
}
}
