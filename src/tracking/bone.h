#ifndef BONE_H
#define BONE_H

#include <Eigen/Geometry>
#include <memory>

namespace pose
{
class Joint;

class Bone
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Bone(std::shared_ptr<Joint> jointStart, std::shared_ptr<Joint> jointEnd, float length);

    const std::shared_ptr<Joint>& getJointStart() const;
    const std::shared_ptr<Joint>& getJointEnd() const;
    const Eigen::Quaternion<double> getOrientation() const;
    float getLength() const;

    void setOrientation(const Eigen::Quaternion<double>& orientation);
    void setLength(float length);

private:
    std::shared_ptr<Joint> m_jointStart;
    std::shared_ptr<Joint> m_jointEnd;
    float m_length;
    Eigen::Quaterniond m_orientation;
};
}

#endif // BONE_H
