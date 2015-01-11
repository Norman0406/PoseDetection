#include "skeletonupperbody.h"
#include "joint.h"
#include "bone.h"
#include <utils/utils.h>

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
    std::shared_ptr<Bone> torsoNeck(new Bone(torso, neck, 0.35f));
    torsoNeck->setOrientation(Eigen::Quaterniond(Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 1))));
    torso->addBone(torsoNeck);

    std::shared_ptr<Bone> neckHead(new Bone(neck, head, 0.27f));
    neckHead->setOrientation(Eigen::Quaterniond(Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 1))));
    neck->addBone(neckHead);

    std::shared_ptr<Bone> neckLeftShoulder(new Bone(neck, leftShoulder, 0.17f));
    neckLeftShoulder->setOrientation(Eigen::Quaterniond(Eigen::AngleAxisd(DEG2RAD(-90), Eigen::Vector3d(0, 0, 1))));
    neck->addBone(neckLeftShoulder);

    std::shared_ptr<Bone> leftShoulderLeftElbow(new Bone(leftShoulder, leftElbow, 0.27f));
    leftShoulderLeftElbow->setOrientation(Eigen::Quaterniond(Eigen::AngleAxisd(DEG2RAD(-90), Eigen::Vector3d(0, 0, 1))));
    leftShoulder->addBone(leftShoulderLeftElbow);

    std::shared_ptr<Bone> leftElbowLeftHand(new Bone(leftElbow, leftHand, 0.35f));
    leftElbow->addBone(leftElbowLeftHand);

    std::shared_ptr<Bone> neckRightShoulder(new Bone(neck, rightShoulder, 0.17f));
    neckRightShoulder->setOrientation(Eigen::Quaterniond(Eigen::AngleAxisd(DEG2RAD(90), Eigen::Vector3d(0, 0, 1))));
    neck->addBone(neckRightShoulder);

    std::shared_ptr<Bone> rightShoulderRightElbow(new Bone(rightShoulder, rightElbow, 0.27f));
    rightShoulderRightElbow->setOrientation(Eigen::Quaterniond(Eigen::AngleAxisd(DEG2RAD(90), Eigen::Vector3d(0, 0, 1))));
    rightShoulder->addBone(rightShoulderRightElbow);

    std::shared_ptr<Bone> rightElbowRightHand(new Bone(rightElbow, rightHand, 0.35f));
    rightElbow->addBone(rightElbowRightHand);

    return torso;
}

void SkeletonUpperBody::setDefaultPose()
{
}
}
