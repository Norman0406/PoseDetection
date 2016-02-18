#include "skeleton.h"
#include "bone.h"
#include <utils/utils.h>

namespace pose
{
Skeleton::Skeleton(unsigned int label, std::shared_ptr<Joint> rootJoint)
    : m_label(label),
      m_rootJoint(rootJoint),
      m_position(0, 0, 0),
      m_isInitialized(false)
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

void Skeleton::setPosition(const cv::Point3f& position)
{
    m_position = position;
}

const cv::Point3f& Skeleton::getPosition() const
{
    return m_position;
}

bool Skeleton::isInitialized() const
{
    return m_isInitialized;
}

void Skeleton::update(const cv::Mat& projectionMatrix)
{
    // compute image position
    cv::Point2f pointImg = Utils::projectPoint(m_position, projectionMatrix);
    m_rootJoint->setPosition(m_position, pointImg);

    // update skeleton hierarchy
    m_rootJoint->update(Eigen::Quaterniond::Identity(), projectionMatrix);

    m_isInitialized = true;
}
}
