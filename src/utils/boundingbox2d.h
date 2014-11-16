#ifndef BOUNDINGBOX2D_H
#define BOUNDINGBOX2D_H

#include <opencv2/opencv.hpp>

namespace pose {
class BoundingBox2D
{
public:
    BoundingBox2D();
    BoundingBox2D(cv::Point minPoint,
                  cv::Point maxPoint,
                  float minDepth,
                  float maxDepth);

    /**
     * @brief The anchor points define the four either vertical or horizontal lines that
     * enclose the object contained within the bounding box. The anchor points are given
     * in X or Y pixel coordinates.
     */
    enum AnchorType {
        AT_LEFT,
        AT_RIGHT,
        AT_TOP,
        AT_BOTTOM
    };

    /**
     * @brief Set the bounding box data and update all information.
     */
    void set(cv::Point minPoint,
             cv::Point maxPoint,
             float minDepth,
             float maxDepth);

    /**
     * @brief Get the minimum point in the upper left corner.
     */
    const cv::Point& getMinPoint() const;

    /**
     * @brief Get the maximum point in the lower right corner.
     */
    const cv::Point& getMaxPoint() const;

    /**
     * @brief Get the minimum depth of the object inside the bounding box.
     */
    float getMinDepth() const;

    /**
     * @brief Get the maximum depth of the object inside the bounding box.
     */
    float getMaxDepth();

    /**
     * @brief Get the bounding box center position.
     */
    const cv::Point& getCenter() const;

    /**
     * @brief Get the bounding box width.
     */
    int getWidth() const;

    /**
     * @brief Get the bounding box height.
     */
    int getHeight() const;

    /**
     * @brief Get the amount of pixels that are covered by the box.
     */
    int getArea() const;

    /**
     * @brief Get the average depth, i.e. the mean between minimum and maximum depth.
     */
    float getAvgDepth() const;

    /**
     * @brief Get the specified anchor position, i.e. the X or Y pixel coordinate of the
     * specified horizontal or vertical bounding box line.
     */
    int getAnchor(AnchorType anchor) const;

    /**
     * @brief Computes the overlapping area between box1 and box2 in pixels
     */
    static int getOverlapArea(const BoundingBox2D& box1,
                              const BoundingBox2D& box2);

    /**
     * @brief Computes the overlapping area between this box and the other box in pixels
     */
    int getOverlapArea(const BoundingBox2D& other) const;

    /**
     * @brief Computes the overlapping factor between this box and the other box, i.e. the
     * percentage of overlapping pixels in this box. If this box is completely inside the
     * other box, the result will be 1.
     */
    float getOverlapFactor(const BoundingBox2D& other) const;

    /**
     * @brief Computes the pixel distance of the specified anchor point between this box
     * and the other box.
     */
    int getAnchorDistance(AnchorType anchor,
                          const BoundingBox2D& other) const;

private:
    // direct information
    cv::Point   m_minPoint;
    cv::Point   m_maxPoint;
    float       m_minDepth;
    float       m_maxDepth;

    // inferred information
    int m_width;
    int m_height;
    int m_area;
    float m_avgDepth;
    cv::Point m_center;
};
}

#endif // BOUNDINGBOX2D_H
