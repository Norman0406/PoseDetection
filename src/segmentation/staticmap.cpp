#include "staticmap.h"

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
        m_filteredDepthMap = cv::Mat(depthMap.rows, depthMap.cols, CV_32F);
        m_shadowMask = cv::Mat(depthMap.rows, depthMap.cols, CV_8U);
        m_background = cv::Mat(depthMap.rows, depthMap.cols, CV_32F);
        m_foreground = cv::Mat(depthMap.rows, depthMap.cols, CV_32F);
        m_foregroundMask = cv::Mat(depthMap.rows, depthMap.cols, CV_8U);
        m_count = cv::Mat(depthMap.rows, depthMap.cols, CV_32F);
        m_tempContour = cv::Mat(depthMap.rows, depthMap.cols, CV_8U);
        m_minSize = depthMap.cols * depthMap.rows / m_minRatio;
        m_background.setTo(0);
        m_count.setTo(0);
    }
    m_filteredDepthMap.setTo(0);
    m_shadowMask.setTo(0);
    m_foreground.setTo(0);
    m_foregroundMask.setTo(0);

    preFilterShadows(depthMap);

    int totalNumPoints = 0;
    int pointsChanged = 0;

    //float filterCollection[24][2];

    if (m_updateFrames == 0) {
        #pragma omp parallel for shared(totalNumPoints, pointsChanged)
        for (int i = 0; i < m_filteredDepthMap.cols; i++) {
            for (int j = 0; j < m_filteredDepthMap.rows; j++) {
                cv::Point point(i, j);
                const float& dist = m_filteredDepthMap.at<float>(point);
                float& background = m_background.at<float>(point);
                unsigned int& count = m_count.at<unsigned int>(point);

                // TODO: account for depth-dependent noise

                if (dist > 0)
                    totalNumPoints++;

                // TODO: prior to updating the background, first look at the neighborhood
                // and check if the neighboring pixels also have a distance > background.
                //bool hasBlack = hasBlackAreas(depthMap, point);
                if (dist > 0 && dist > background) {// && !hasBlack) {
                    //background = dist;

                    //background = ((background * count) / (count + 1)) + (dist * (1 / (count + 1)));
                    //background = ((dist * count) / (count + 1)) + (background * (1 / (count + 1)));

                    // cumulative moving average
                    //background = background + ((dist - background) / (count + 1));

                    // modified moving average
                    background = (background * count + dist) / (count + 1);

                    pointsChanged++;
                    count++;
                }
                /*else if (dist == 0) {
                    // apply filtering

                    // clear filter collection
                    for (int k = 0; k < 24; k++)
                        for (int l = 0; l < 2; l++)
                            filterCollection[k][l] = 0;

                    int innerBandCount = 0;
                    int outerBandCount = 0;

                    // search in 2 bands
                    for (int k = -2; k <= 2; k++) {
                        for (int l = -2; l <= 2; l++) {
                            if (k == 0 && l == 0)
                                continue;

                            cv::Point bandPoint(i + k, j + l);
                            if (bandPoint.x < 0 || bandPoint.x >= depthMap.cols ||
                                bandPoint.y < 0 || bandPoint.y >= depthMap.rows)
                                continue;

                            const float& bandDist = depthMap.at<float>(bandPoint);
                            const float bandBackground = m_background.at<float>(bandPoint);

                            if (bandDist == 0 || bandBackground == 0 || bandDist < bandBackground)
                                continue;

                            for (int m = 0; m < 24; m++) {
                                if (filterCollection[m][0] == bandDist) {
                                    filterCollection[m][1]++;
                                    break;
                                }
                                else if (filterCollection[m][0] == 0) {
                                    filterCollection[m][0] = bandDist;
                                    filterCollection[m][1]++;
                                    break;
                                }
                            }

                            if (k != 2 && k != -2 && l != 2 && l != -2)
                                innerBandCount++;
                            else
                                outerBandCount++;
                        }
                    }

                    const int innerBandThreshold = 2;
                    const int outerBandThreshold = 4;

                    if (innerBandCount >= innerBandThreshold || outerBandCount >= outerBandThreshold) {
                        short frequency = 0;
                        float newDepth = 0;

                        for (int m = 0; m < 24; m++) {
                            if (filterCollection[m][0] == 0)
                                break;
                            if (filterCollection[m][1] > frequency) {
                                newDepth = filterCollection[m][0];
                                frequency = filterCollection[m][1];
                            }
                        }

                        background = newDepth;
                        //background = (background * count + newDepth) / (count + 1);
                        //count++;
                    }
                    else {
                        if (dist > background) {
                            background = dist;
                            //background = (background * count + dist) / (count + 1);
                            //count++;
                        }
                    }
                }*/

                if (dist > 0 && (background - dist) > m_foregroundDistance) {
                    float& foreground = m_foreground.at<float>(point);
                    uchar& foregroundMask = m_foregroundMask.at<uchar>(point);
                    foreground = dist;
                    foregroundMask = 255;
                }
            }
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

    filterContours();
}

void StaticMap::preFilterShadows(const cv::Mat& depthMap)
{
    m_shadowMask.setTo(0);
    for (int i = 0; i < depthMap.cols; i++) {
        for (int j = 0; j < depthMap.rows; j++) {
            cv::Point point(i, j);
            const float& dist = depthMap.at<float>(point);
            uchar& shadow = m_shadowMask.at<uchar>(point);

            if (dist == 0)
                shadow = 255;
        }
    }

    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2));
    cv::Mat temp;// = m_shadowMask.clone();
    cv::dilate(m_shadowMask, temp, element);

    m_shadowContours.clear();
    cv::findContours(temp, m_shadowContours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    temp.setTo(255);

    // TODO: merge all non-black contours with black contours

    for (int i = 0; i < (int)m_shadowContours.size(); i++) {
        //cv::drawContours(m_shadowMask, m_shadowContours, i, cv::Scalar::all(0), 1, 8, hierarchy);
        cv::drawContours(temp, m_shadowContours, i, cv::Scalar::all(0), CV_FILLED, 8);
    }

    cv::imshow("Result", temp);

    //cv::imshow("Shadows", m_shadowMask);

    /*cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::Mat temp;
    cv::erode(m_shadowMask, temp, element);
    depthMap.copyTo(m_filteredDepthMap, temp);*/

    depthMap.copyTo(m_filteredDepthMap, temp);
    cv::imshow("Filtered", m_filteredDepthMap * 0.2f);
}

bool StaticMap::hasBlackAreas(const cv::Mat& depthMap, const cv::Point& point) const
{
    int numBlackPoints = 0;

    int windowSize = 1;
    for (int i = -windowSize; i <= windowSize; i++) {
        for (int j = -windowSize; j <= windowSize; j++) {
            // skip the center pixel
            if (i == 0 && j == 0)
                continue;

            cv::Point point(i + point.x, j + point.y);

            if (point.x < 0 || point.x >= depthMap.cols ||
                point.y < 0 || point.y >= depthMap.rows)
                continue;

            const float& dist = depthMap.at<float>(point);

            if (dist == 0)
                numBlackPoints++;
        }
    }

    return numBlackPoints > 0;
}

float StaticMap::getNeighboringDistance(const cv::Mat& depthMap, const cv::Point& point) const
{
    float meanDistance = 0;
    int numPoints = 0;

    int windowSize = 1;
    for (int i = -windowSize; i <= windowSize; i++) {
        for (int j = -windowSize; j <= windowSize; j++) {
            // skip the center pixel
            if (i == 0 && j == 0)
                continue;

            cv::Point point(i + point.x, j + point.y);

            if (point.x < 0 || point.x >= depthMap.cols ||
                point.y < 0 || point.y >= depthMap.rows)
                continue;

            const float& dist = depthMap.at<float>(point);

            if (dist > 0) {
                meanDistance += dist;
                numPoints++;
            }
        }
    }

    if (numPoints == 0)
        return 0;

    meanDistance /= numPoints;
    return meanDistance;
}

void StaticMap::filterContours()
{
    // perform an opening to supress noise
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
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
