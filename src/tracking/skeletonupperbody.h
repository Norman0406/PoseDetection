#ifndef SKELETONUPPERBODY_H
#define SKELETONUPPERBODY_H

#include "skeleton.h"

namespace pose
{
class SkeletonUpperBody
        : public Skeleton
{
public:
    SkeletonUpperBody(unsigned int label);

    void setDefaultPose();

private:
    static std::shared_ptr<Joint> create();
};
}

#endif // SKELETONUPPERBODY_H
