#include "bone.h"
#include "joint.h"
#include <utils/utils.h>

namespace pose
{
Bone::Bone(std::shared_ptr<Joint> jointStart, std::shared_ptr<Joint> jointEnd, float length, bool isFixed)
    : m_jointStart(jointStart),
      m_jointEnd(jointEnd),
      m_orientation(Eigen::Quaterniond::Identity()),
      m_length(length),
      m_isFixed(isFixed)
{
}

const std::shared_ptr<Joint>& Bone::getJointStart() const
{
    return m_jointStart;
}

const std::shared_ptr<Joint>& Bone::getJointEnd() const
{
    return m_jointEnd;
}

const Eigen::Quaternion<double> Bone::getOrientation() const
{
    return m_orientation;
}

float Bone::getLength() const
{
    return m_length;
}

bool Bone::isFixed() const
{
    return m_isFixed;
}

void Bone::setOrientation(const Eigen::Quaternion<double>& orientation)
{
    m_orientation = orientation;
}

void Bone::setLength(float length)
{
    m_length = length;
}

void Bone::update(const Eigen::Quaterniond& globalQuat, const cv::Mat& projectionMatrix)
{
    // compute global orientation
    Eigen::Quaterniond orientation = globalQuat * m_orientation;

    // compute new rotated position
    Eigen::Quaterniond pointQuat(0, 0, m_length, 0);
    Eigen::Quaterniond result = orientation * pointQuat * orientation.conjugate();
    cv::Point3f newPosition = m_jointStart->getPosition3d() +
            cv::Point3f((float)result.x(), (float)result.y(), (float)result.z());

    // compute image position
    cv::Point2f pointImg = Utils::projectPoint(newPosition, projectionMatrix);
    m_jointEnd->setPosition(newPosition, pointImg);

    // then update target joint
    m_jointEnd->update(orientation, projectionMatrix);
}
}
