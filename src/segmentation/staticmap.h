#ifndef STATICMAP_H
#define STATICMAP_H

#include <opencv2/opencv.hpp>

// TODO: compute normals and cluster normals by their direction to filter out walls and the floor

// TODO: create a "shadow" map, i.e. a map indicating which black areas are adjacent to a foreground
// object and don't update the background model at areas that are inside the shadow map

// TODO: when tracking connected components, add back components to the background model that have not
// been identified as a human body.

// TODO: when a pixel does not have a valid depth value for a certain amount of time, the background pixel
// will lose its value

namespace pose {
class StaticMap
{
public:
    StaticMap();
    ~StaticMap();

    /**
     * @brief Sets the delay of frames after which the background will be updated again after a
     * reset happened. This is to account for additional motion that might happen after a reset
     * has been triggered.
     */
    void setUpdateDelayFrames(int frames);

    /**
     * @brief Determines the time after a reset at which the background model is supposed to be
     * stable. I.e., if the number of points that are updated at the background model falls below
     * a certain ratio, the background model is stable.
     */
    void setBackgroundLockedRatio(float ratio);

    /**
     * @brief Sets the ratio of points that have been updated in the background model to determine
     * that the background model should be resetted.
     */
    void setBackgroundResetRatio(float ratio);

    /**
     * @brief Sets the minimum distance that a pixel should have from the background model to be
     * considered part of the foreground.
     */
    void setForegroundDistance(float distance);

    void setMinRatio(int minRatio);

    const cv::Mat& getBackground() const;
    const cv::Mat& getForeground() const;

    void process(const cv::Mat& depthMap);

private:
    void reset();
    void filterContours();

    cv::Mat m_background;
    cv::Mat m_foreground;
    cv::Mat m_foregroundMask;
    cv::Mat m_count;
    std::vector<std::vector<cv::Point>> m_contours;
    cv::Mat m_tempContour;

    int     m_updateFrames;
    int     m_updateDelayFrames;
    float   m_backgroundResetRatio;
    bool    m_backgroundLocked;
    float   m_backgroundLockedRatio;
    float   m_foregroundDistance;
    int     m_minSize;
    int     m_minRatio;
};
}

#endif // STATICMAP_H
