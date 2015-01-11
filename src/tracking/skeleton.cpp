#include "skeleton.h"

namespace pose
{
Skeleton::Skeleton(unsigned int label, std::shared_ptr<Joint> rootJoint)
    : m_label(label),
      m_rootJoint(rootJoint),
      m_position(0, 0, 2)
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

void Skeleton::update(const cv::Mat& projectionMatrix)
{
    // compute image position
    cv::Mat point(4, 1, CV_32F);
    point.ptr<float>(0)[0] = m_position.x;
    point.ptr<float>(0)[1] = m_position.y;
    point.ptr<float>(0)[2] = m_position.z;
    point.ptr<float>(0)[3] = 1;

    cv::Mat pointImgMat = projectionMatrix * point;
    cv::Point2f pointImg(pointImgMat.ptr<float>(0)[0] / pointImgMat.ptr<float>(0)[2],
            pointImgMat.ptr<float>(0)[1] / pointImgMat.ptr<float>(0)[2]);
    m_rootJoint->setPosition(m_position, pointImg);

    // update skeleton hierarchy
    m_rootJoint->update(Eigen::Quaterniond::Identity(), projectionMatrix);
}
}
