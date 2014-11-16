#ifndef BOUNDINGBOX3D_H
#define BOUNDINGBOX3D_H

#include <opencv2/opencv.hpp>

namespace pose {
class BoundingBox3D
{
public:
    BoundingBox3D();
    BoundingBox3D(cv::Point3f minPoint,
                  cv::Point3f maxPoint);

    /**
     * @brief The anchor points define the six planes that enclose the object contained
     * within the bounding box. The anchor points are given in X, Y or Z world coordinates.
     */
    enum AnchorType {
        AT_LEFT,
        AT_RIGHT,
        AT_TOP,
        AT_BOTTOM,
        AT_BACK,
        AT_FRONT
    };

    /**
     * @brief Set the bounding box data and update all information.
     */
    void set(cv::Point3f minPoint,
             cv::Point3f maxPoint);

    /**
     * @brief Get the minimum point in the upper left corner.
     */
    const cv::Point3f& getMinPoint() const;

    /**
     * @brief Get the maximum point in the lower right corner.
     */
    const cv::Point3f& getMaxPoint() const;

    /**
     * @brief Get the bounding box center position.
     */
    const cv::Point3f& getCenter() const;

    /**
     * @brief Get the bounding box width.
     */
    const cv::Point3f& getSize() const;

    /**
     * @brief Get the metric space that is covered by the box.
     */
    float getArea() const;

    /**
     * @brief Get the specified anchor position, i.e. the X, Y or Z world coordinate of the
     * specified bounding box plane.
     */
    float getAnchor(AnchorType anchor) const;

private:
    // direct information
    cv::Point3f m_minPoint;
    cv::Point3f m_maxPoint;

    // inferred information
    cv::Point3f m_size;
    float       m_area;
    cv::Point3f m_center;
};
}

#endif // BOUNDINGBOX3D_H
