#include "skeleton.h"

namespace pose
{
Skeleton::Skeleton(unsigned int label, std::shared_ptr<Joint> rootJoint)
    : m_label(label),
      m_rootJoint(rootJoint)
{
}

unsigned int Skeleton::getLabel() const
{
    return m_label;
}

const std::shared_ptr<Joint>& Skeleton::getRootJoint() const
{
    return m_rootJoint;
}

const Joint* Skeleton::getJoint(Joint::JointType type) const
{
    return m_rootJoint->getJoint(type);
}

void Skeleton::update()
{
}
}
