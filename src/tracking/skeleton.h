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

    void update();

    unsigned int getLabel() const;
    const std::shared_ptr<Joint>& getRootJoint() const;
    const Joint* getJoint(Joint::JointType type) const;

private:
    std::shared_ptr<Joint> m_rootJoint;
    unsigned int m_label;
};
}

#endif // SKELETON_H
