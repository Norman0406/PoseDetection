#include "fitting.h"
#include "skeletonupperbody.h"
#include "joint.h"
#include "bone.h"
#include <utils/utils.h>

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
    // create and update the skeleton list
    create(labelMap);

    // update each skeleton to fit to its user
    update(depthMap, labelMap, pointCloud, projectionMatrix);

    // debug drawing
    draw(depthMap, labelMap);
}

void Fitting::create(const cv::Mat& labelMap)
{
    // find labels and create new skeletons
    std::vector<int> labels;
    for (int i = 0; i < labelMap.rows; i++) {
        const unsigned int* labelRow = labelMap.ptr<unsigned int>(i);
        for (int j = 0; j < labelMap.cols; j++) {
            unsigned int label = labelRow[j];

            if (label > 0) {
                if (std::find(labels.begin(), labels.end(), label) == labels.end())
                    labels.push_back(label);

                // create a new skeleton if the label is not yet available
                if (m_skeletons.find(label) == m_skeletons.end())
                    m_skeletons[label] = std::shared_ptr<Skeleton>(new SkeletonUpperBody(label));
            }
        }
    }

    // remove old skeletons
    for (auto it = m_skeletons.begin(); it != m_skeletons.end();) {
        unsigned int label = it->first;
        if (std::find(labels.begin(), labels.end(), label) == labels.end())
            it = m_skeletons.erase(it);
        else
            it++;
    }
}

void Fitting::update(const cv::Mat& depthMap, const cv::Mat& labelMap, const cv::Mat& pointCloud, const cv::Mat& projectionMatrix)
{
    for (auto it = m_skeletons.begin(); it != m_skeletons.end(); it++) {
        // NOTE: this loop might be parallelized

        const std::shared_ptr<Skeleton>& skeleton = it->second;
        unsigned int label = skeleton->getLabel();

        cv::Mat userDepthMap(depthMap.rows, depthMap.cols, depthMap.type());
        cv::Mat userPointCloud(pointCloud.rows, pointCloud.cols, pointCloud.type());
        userDepthMap.setTo(0);
        userPointCloud.setTo(0);

        float m100 = 0, m010 = 0, m001 = 0, m000 = 0;

        // create an image that contains only pixels for the selected skeleton
        for (int i = 0; i < depthMap.rows; i++) {
            const unsigned int* labelRow = labelMap.ptr<unsigned int>(i);
            const float* depthRow = depthMap.ptr<float>(i);
            const cv::Vec3f* pointsRow = pointCloud.ptr<cv::Vec3f>(i);
            float* userDepthRow = userDepthMap.ptr<float>(i);
            cv::Vec3f* userPointsRow = userPointCloud.ptr<cv::Vec3f>(i);

            for (int j = 0; j < depthMap.cols; j++) {
                if (labelRow[j] == label) {
                    const float& depthValue = depthRow[j];
                    const cv::Vec3f& pointsValue = pointsRow[j];

                    userDepthRow[j] = depthValue;
                    userPointsRow[j] = depthValue;

                    // compute moments
                    m100 += pointsValue[0];
                    m010 += pointsValue[1];
                    m001 += pointsValue[2];
                    m000 += 1;
                }
            }
        }

        if (m000 > 0) {
            cv::Point3f centerOfMass(m100 / m000, m010 / m000, m001 / m000);

            // run skeleton fitting
            runFitting(userDepthMap, userPointCloud, skeleton, centerOfMass, projectionMatrix);
        }
    }
}

void Fitting::runFitting(const cv::Mat& depthMap, const cv::Mat& pointCloud, std::shared_ptr<Skeleton> skeleton, cv::Point3f centerOfMass, const cv::Mat& projectionMatrix)
{
    skeleton->setPosition(centerOfMass);
    skeleton->update(projectionMatrix);
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
        cv::circle(dispImg, skeleton->getRootJoint()->getPosition2d(), 4, Utils::getLabelColor(skeleton->getLabel()), -1);
    }    

    cv::imshow("Skeleton", dispImg);
}

void Fitting::drawJoint(const std::shared_ptr<Joint>& joint, cv::Mat& dispImg)
{
    cv::Scalar color(255, 255, 255, 255);
    color *= joint->getConfidence();
    cv::circle(dispImg, joint->getPosition2d(), 4, color);

    auto bones = joint->getBones();
    for (size_t i = 0; i < bones.size(); i++) {
        const std::shared_ptr<Bone>& bone = bones[i];

        cv::line(dispImg, joint->getPosition2d(), bone->getJointEnd()->getPosition2d(), color);
        drawJoint(bone->getJointEnd(), dispImg);
    }
}
}
