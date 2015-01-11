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

void Bone::update(const Eigen::Quaterniond& globalQuat, const cv::Mat& projectionMatrix)
{
    // compute global orientation
    Eigen::Quaterniond orientation = globalQuat * m_orientation;

    // compute new rotated position
    Eigen::Quaterniond pointQuat(0, 0, m_length, 0);
    Eigen::Quaterniond result = orientation * pointQuat * orientation.conjugate();
    cv::Point3f newPosition = m_jointStart->getPosition3d() +
            cv::Point3f((float)result.x(), (float)result.y(), (float)result.z());

    cv::Mat point(4, 1, CV_32F);
    point.ptr<float>(0)[0] = newPosition.x;
    point.ptr<float>(0)[1] = newPosition.y;
    point.ptr<float>(0)[2] = newPosition.z;
    point.ptr<float>(0)[3] = 1;

    // compute image position
    cv::Mat pointImgMat = projectionMatrix * point;
    cv::Point2f pointImg(pointImgMat.ptr<float>(0)[0] / pointImgMat.ptr<float>(0)[2],
            pointImgMat.ptr<float>(0)[1] / pointImgMat.ptr<float>(0)[2]);
    m_jointEnd->setPosition(newPosition, pointImg);

    // then update target joint
    m_jointEnd->update(orientation, projectionMatrix);
}
}
