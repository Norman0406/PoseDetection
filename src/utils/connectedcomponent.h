#ifndef CONNECTEDCOMPONENT_H
#define CONNECTEDCOMPONENT_H

#include <opencv2/opencv.hpp>
#include "boundingbox2d.h"
#include "boundingbox3d.h"

namespace pose {
/*class ConnectedComponent
{
public:
    ConnectedComponent(unsigned int id);

    unsigned int getId() const;
    const cv::Point& getCenterOfMass() const;
    float getCenterDepth() const;
    const BoundingBox2D& getBoundingBox2D() const;
    const BoundingBox3D& getBoundingBox3D() const;
    int getArea() const;
    const std::vector<unsigned int>& getNearbyIds() const;

private:
    unsigned int    m_id;
    cv::Point       m_centerOfMass;
    float           m_centerDepth;
    BoundingBox2D   m_boundingBox2d;
    BoundingBox3D   m_boundingBox3d;
    int             m_area;
    std::vector<unsigned int> nearbyIds;
};*/
}

#endif // CONNECTEDCOMPONENT_H
