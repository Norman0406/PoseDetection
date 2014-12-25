#ifndef JOINT_H
#define JOINT_H

#include <opencv2/opencv.hpp>
#include <memory>

namespace pose
{
class Bone;

class Joint
{
public:
    enum JointType
    {
        JT_HEAD,
        JT_NECK,
        JT_LEFTSHOULDER,
        JT_LEFTELBOW,
        JT_LEFTHAND,
        JT_RIGHTSHOULDER,
        JT_RIGHTELBOW,
        JT_RIGHTHAND,
        JT_TORSO,
        JT_HIPS,
        JT_LEFTHIP,
        JT_LEFTKNEE,
        JT_LEFTFOOT,
        JT_RIGHTHIP,
        JT_RIGHTKNEE,
        JT_RIGHTFOOT
    };

    Joint(JointType type);
    ~Joint();

    const JointType& getJointType() const;
    const cv::Point3f& getPosition3d() const;
    const cv::Point2f& getPosition2d() const;
    float getConfidence() const;
    const std::vector<std::shared_ptr<Bone>>& getBones() const;

    bool addBone(std::shared_ptr<Bone> bone);
    void setPosition(const cv::Point3f& pos3d, const cv::Point2f& pos2d);
    void setConfidence(float confidence);

    const Joint* getJoint(JointType type) const;

private:
    JointType m_type;
    std::vector<std::shared_ptr<Bone>> m_bones;
    cv::Point3f m_position3d;
    cv::Point2f m_position2d;
    float m_confidence;
};
}

#endif // JOINT_H
