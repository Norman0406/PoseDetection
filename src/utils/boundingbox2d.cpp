#include "boundingbox2d.h"

BoundingBox2D::BoundingBox2D()
{
    set(cv::Point(0, 0), cv::Point(0, 0), 0, 0);
}

BoundingBox2D::BoundingBox2D(cv::Point minPoint, cv::Point maxPoint, float minDepth, float maxDepth)
{
    set(minPoint, maxPoint, minDepth, maxDepth);
}

void BoundingBox2D::set(cv::Point minPoint, cv::Point maxPoint, float minDepth, float maxDepth)
{
    m_minPoint = minPoint;
    m_maxPoint = maxPoint;
    m_minDepth = minDepth;
    m_maxDepth = maxDepth;

    assert(m_minPoint.x <= m_maxPoint.x);
    assert(m_minPoint.y <= m_maxPoint.y);
    assert(m_minDepth <= m_maxDepth);

    // update inferred data
    m_width = m_maxPoint.x - m_minPoint.x;
    m_height = m_maxPoint.y - m_minPoint.y;
    m_center = cv::Point(m_minPoint.x + (m_width / 2), m_minPoint.y + (m_height / 2));
    m_area = (getAnchor(AT_RIGHT) - getAnchor(AT_LEFT)) *
            (getAnchor(AT_BOTTOM) - getAnchor(AT_TOP));
    m_avgDepth = m_minDepth + ((m_maxDepth - m_minDepth) / 2.0f);
}

const cv::Point& BoundingBox2D::getMinPoint() const
{
    return m_minPoint;
}

const cv::Point& BoundingBox2D::getMaxPoint() const
{
    return m_maxPoint;
}

const cv::Point& BoundingBox2D::getCenter() const
{
    return m_center;
}

float BoundingBox2D::getMinDepth() const
{
    return m_minDepth;
}

float BoundingBox2D::getMaxDepth()
{
    return m_maxDepth;
}

int BoundingBox2D::getWidth() const
{
    return m_width;
}

int BoundingBox2D::getHeight() const
{
    return m_height;
}

int BoundingBox2D::getArea() const
{
    return m_area;
}

float BoundingBox2D::getAvgDepth() const
{
    return m_avgDepth;
}

int BoundingBox2D::getAnchor(AnchorType anchor) const
{
    switch (anchor) {
    case AT_LEFT:
        return m_minPoint.x;
    case AT_RIGHT:
        return m_maxPoint.x;
    case AT_TOP:
        return m_minPoint.y;
    case AT_BOTTOM:
        return m_maxPoint.y;
    }
    return -1;
}

int BoundingBox2D::getOverlapArea(const BoundingBox2D& box1, const BoundingBox2D& box2)
{
    const int top = std::max(box1.getAnchor(AT_TOP), box2.getAnchor(AT_TOP));
    const int left = std::max(box1.getAnchor(AT_LEFT), box2.getAnchor(AT_LEFT));
    const int right = std::min(box1.getAnchor(AT_RIGHT), box2.getAnchor(AT_RIGHT));
    const int bottom = std::min(box1.getAnchor(AT_BOTTOM), box2.getAnchor(AT_BOTTOM));

    return (right - left) + (bottom - top);
}

int BoundingBox2D::getOverlapArea(const BoundingBox2D& other) const
{
    return BoundingBox2D::getOverlapArea(*this, other);
}

float BoundingBox2D::getOverlapFactor(const BoundingBox2D& other) const
{
    int area = getOverlapArea(other);
    return area / (float)getArea();
}

int BoundingBox2D::getAnchorDistance(AnchorType anchor, const BoundingBox2D& other) const
{
    return abs(getAnchor(anchor) - other.getAnchor(anchor));
}
