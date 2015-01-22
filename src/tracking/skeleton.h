#ifndef SKELETON_HH
#define SKELETON_HH

#include <memory>
#include "joint.h"

namespace pose
{
class Skeleton
{
public:
    Skeleton(unsigned int label, std::shared_ptr<Joint> rootJoint);

    virtual void setDefaultPose() = 0;

    void setPosition(const cv::Point3f& position);
    const cv::Point3f& getPosition() const;
    void update(const cv::Mat& projectionMatrix);
    bool isInitialized() const;

    unsigned int getLabel() const;
    const std::shared_ptr<Joint>& getRootJoint() const;
    const Joint* getJoint(Joint::JointType type) const;

    float getEnergy() const;

private:
    std::shared_ptr<Joint> m_rootJoint;
    unsigned int m_label;
    cv::Point3f m_position;
    bool m_isInitialized;
};
}

#endif // SKELETON_H
