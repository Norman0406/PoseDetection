#include "skeletonupperbody.h"
#include "joint.h"
#include "bone.h"

namespace pose
{
SkeletonUpperBody::SkeletonUpperBody(unsigned int label)
    : Skeleton(label, create())
{
}

std::shared_ptr<Joint> SkeletonUpperBody::create()
{
    // create joints
    std::shared_ptr<Joint> torso(new Joint(Joint::JT_TORSO));
    std::shared_ptr<Joint> neck(new Joint(Joint::JT_NECK));
    std::shared_ptr<Joint> head(new Joint(Joint::JT_HEAD));
    std::shared_ptr<Joint> leftShoulder(new Joint(Joint::JT_LEFTSHOULDER));
    std::shared_ptr<Joint> leftElbow(new Joint(Joint::JT_LEFTELBOW));
    std::shared_ptr<Joint> leftHand(new Joint(Joint::JT_LEFTHAND));
    std::shared_ptr<Joint> rightShoulder(new Joint(Joint::JT_RIGHTSHOULDER));
    std::shared_ptr<Joint> rightElbow(new Joint(Joint::JT_RIGHTELBOW));
    std::shared_ptr<Joint> rightHand(new Joint(Joint::JT_RIGHTHAND));

    // create bones
    std::shared_ptr<Bone> torsoNeck(new Bone(torso, neck, 1.0f));
    torso->addBone(torsoNeck);

    std::shared_ptr<Bone> neckHead(new Bone(neck, head, 1.0f));
    neck->addBone(neckHead);

    std::shared_ptr<Bone> neckLeftShoulder(new Bone(neck, leftShoulder, 1.0f));
    neck->addBone(neckLeftShoulder);

    std::shared_ptr<Bone> leftShoulderLeftElbow(new Bone(leftShoulder, leftElbow, 1.0f));
    leftShoulder->addBone(leftShoulderLeftElbow);

    std::shared_ptr<Bone> leftElbowLeftHand(new Bone(leftElbow, leftHand, 1.0f));
    leftElbow->addBone(leftElbowLeftHand);

    std::shared_ptr<Bone> neckRightShoulder(new Bone(neck, rightShoulder, 1.0f));
    neck->addBone(neckRightShoulder);

    std::shared_ptr<Bone> rightShoulderRightElbow(new Bone(rightShoulder, rightElbow, 1.0f));
    rightShoulder->addBone(rightShoulderRightElbow);

    std::shared_ptr<Bone> rightElbowRightHand(new Bone(rightElbow, rightHand, 1.0f));
    rightElbow->addBone(rightElbowRightHand);

    return torso;
}

void SkeletonUpperBody::setDefaultPose()
{
}
}
