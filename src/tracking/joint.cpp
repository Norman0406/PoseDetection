#include "joint.h"
#include "bone.h"

namespace pose
{
Joint::Joint(Joint::JointType type)
    : m_type(type),
      m_position3d(0, 0, 0),
      m_position2d(0, 0),
      m_confidence(1)
{
}

Joint::~Joint()
{
    m_bones.clear();
}

const Joint::JointType& Joint::getJointType() const
{
    return m_type;
}

const cv::Point3f& Joint::getPosition3d() const
{
    return m_position3d;
}

const cv::Point2f& Joint::getPosition2d() const
{
    return m_position2d;
}

float Joint::getConfidence() const
{
    return m_confidence;
}

const std::vector<std::shared_ptr<Bone>>& Joint::getBones() const
{
    return m_bones;
}

bool Joint::addBone(std::shared_ptr<Bone> bone)
{
    if (bone->getJointStart().get() != this)
        return false;

    m_bones.push_back(bone);
    return true;
}

void Joint::setPosition(const cv::Point3f& pos3d, const cv::Point2f& pos2d)
{
    m_position3d = pos3d;
    m_position2d = pos2d;
}

void Joint::setConfidence(float confidence)
{
    m_confidence = confidence;
}

const Joint* Joint::getJoint(JointType type) const
{
    // check this type
    if (m_type == type)
        return this;

    // check all child bones
    for (size_t i = 0; i < m_bones.size(); i++) {
        const std::shared_ptr<Bone>& bone = m_bones[i];
        const Joint* joint = bone->getJointEnd()->getJoint(type);

        if (joint)
            return joint;
    }

    // not found
    return 0;
}

void Joint::update(const Eigen::Quaterniond& globalQuat, const cv::Mat& projectionMatrix)
{
    for (size_t i = 0; i < m_bones.size(); i++)
        m_bones[i]->update(globalQuat, projectionMatrix);
}
}
