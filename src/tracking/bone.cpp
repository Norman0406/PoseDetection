#include "bone.h"
#include "joint.h"

namespace pose
{
Bone::Bone(std::shared_ptr<Joint> jointStart, std::shared_ptr<Joint> jointEnd, float length)
    : m_jointStart(jointStart),
      m_jointEnd(jointEnd),
      m_orientation(Eigen::Quaterniond::Identity()),
      m_length(length)
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

void Bone::setOrientation(const Eigen::Quaternion<double>& orientation)
{
    m_orientation = orientation;
}

void Bone::setLength(float length)
{
    m_length = length;
}
}
