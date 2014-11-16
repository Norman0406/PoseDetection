#include "boundingbox3d.h"

BoundingBox3D::BoundingBox3D()
{
    set(cv::Point3f(0, 0, 0), cv::Point3f(0, 0, 0));
}

BoundingBox3D::BoundingBox3D(cv::Point3f minPoint, cv::Point3f maxPoint)
{
    set(minPoint, maxPoint);
}

void BoundingBox3D::set(cv::Point3f minPoint, cv::Point3f maxPoint)
{
    m_minPoint = minPoint;
    m_maxPoint = maxPoint;

    assert(m_minPoint.x <= m_maxPoint.x);
    assert(m_minPoint.y <= m_maxPoint.y);
    assert(m_minPoint.z <= m_maxPoint.z);

    // update inferred data
    m_size = cv::Point3f(m_maxPoint.x - m_minPoint.x, m_maxPoint.y - m_minPoint.y, m_maxPoint.z - m_minPoint.z);
    m_center = cv::Point3f(m_minPoint.x + (m_size.x / 2.0f),
                           m_minPoint.y + (m_size.y / 2.0f),
                           m_minPoint.z + (m_size.z / 2.0f));
    m_area = (getAnchor(AT_RIGHT) - getAnchor(AT_LEFT)) *
            (getAnchor(AT_BOTTOM) - getAnchor(AT_TOP)) *
            (getAnchor(AT_BACK) - getAnchor(AT_FRONT));
}

const cv::Point3f& BoundingBox3D::getMinPoint() const
{
    return m_minPoint;
}

const cv::Point3f& BoundingBox3D::getMaxPoint() const
{
    return m_maxPoint;
}

const cv::Point3f& BoundingBox3D::getCenter() const
{
    return m_center;
}

const cv::Point3f& BoundingBox3D::getSize() const
{
    return m_size;
}

float BoundingBox3D::getArea() const
{
    return m_area;
}

float BoundingBox3D::getAnchor(AnchorType anchor) const
{
    switch (anchor) {
    case AT_LEFT:
        return m_minPoint.x;
    case AT_RIGHT:
        return m_maxPoint.x;
    case AT_TOP:
        return m_maxPoint.y;
    case AT_BOTTOM:
        return m_minPoint.y;
    case AT_BACK:
        return m_maxPoint.z;
    case AT_FRONT:
        return m_minPoint.z;
    }
    return -1;
}
