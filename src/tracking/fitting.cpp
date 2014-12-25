#include "fitting.h"
#include "skeletonupperbody.h"
#include "joint.h"
#include "bone.h"

namespace pose
{
Fitting::Fitting()
{
}

Fitting::~Fitting()
{
    m_skeletons.clear();
}

void Fitting::process(const cv::Mat& depthMap,
                      const cv::Mat& pointCloud,
                      const cv::Mat& labelMap,
                      const cv::Mat& projectionMatrix)
{
    create(depthMap, labelMap);
    update(depthMap, labelMap);
    draw(depthMap, labelMap);
}

void Fitting::create(const cv::Mat& depthMap, const cv::Mat& labelMap)
{
    // find labels and create new skeletons
    std::vector<int> labels;
    for (int i = 0; i < labelMap.rows; i++) {
        const unsigned int* labelRow = labelMap.ptr<unsigned int>(i);
        for (int j = 0; j < labelMap.cols; j++) {
            unsigned int label = labelRow[j];

            if (std::find(labels.begin(), labels.end(), label) == labels.end())
                labels.push_back(label);

            // create a new skeleton if the label is not yet available
            if (m_skeletons.find(label) == m_skeletons.end())
                m_skeletons[label] = std::shared_ptr<Skeleton>(new SkeletonUpperBody(label));
        }
    }

    // remove old skeletons
    for (size_t i = 0; i < labels.size(); i++) {
        unsigned int label = labels[i];
        if (m_skeletons.find(label) == m_skeletons.end())
            m_skeletons.erase(label);
    }
}

void Fitting::update(const cv::Mat& depthMap, const cv::Mat& labelMap)
{
    for (auto it = m_skeletons.begin(); it != m_skeletons.end(); it++) {
        const std::shared_ptr<Skeleton>& skeleton = it->second;

        skeleton->update();
    }
}

void Fitting::draw(const cv::Mat& depthMap, const cv::Mat& labelMap)
{
    cv::Mat dispImg(labelMap.rows, labelMap.cols, CV_8UC3);
    dispImg.setTo(0);

    // draw depth values
    for (int i = 0; i < labelMap.rows; i++) {
        const float* depthRow = depthMap.ptr<float>(i);
        const unsigned int* labelRow = labelMap.ptr<unsigned int>(i);
        cv::Vec3b* dispRow = dispImg.ptr<cv::Vec3b>(i);

        for (int j = 0; j < labelMap.cols; j++) {
            float depth = depthRow[j];
            unsigned int label = labelRow[j];

            if (label > 0) {
                uchar value = (uchar)(depth * 255.0f * 0.2f);
                dispRow[j] = cv::Vec3b(value, value, value);
            }
        }
    }

    // draw skeletons
    for (auto it = m_skeletons.begin(); it != m_skeletons.end(); it++) {
        const std::shared_ptr<Skeleton>& skeleton = it->second;
        drawJoint(skeleton->getRootJoint(), dispImg);
    }

    cv::imshow("Skeleton", dispImg);
}

void Fitting::drawJoint(const std::shared_ptr<Joint>& joint, cv::Mat& dispImg)
{
    cv::circle(dispImg, joint->getPosition2d(), 3, cv::Scalar(255, 255, 255));

    auto bones = joint->getBones();
    for (size_t i = 0; i < bones.size(); i++) {
        const std::shared_ptr<Bone>& bone = bones[i];

        cv::line(dispImg, joint->getPosition2d(), bone->getJointEnd()->getPosition2d(), cv::Scalar(255, 255, 255));
        drawJoint(bone->getJointEnd(), dispImg);
    }
}
}
