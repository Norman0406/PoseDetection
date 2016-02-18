#include "connectedcomponentlabeling.h"
#include <utils/utils.h>
#include <stack>
#include <queue>

namespace pose
{
ConnectedComponentLabeling::ConnectedComponentLabeling()
    : Module("ConnectedComponentLabeling"),
      m_maxDistance(0.1f)
{
    setMaxDistance(0.3f);
}

ConnectedComponentLabeling::~ConnectedComponentLabeling()
{
}

void ConnectedComponentLabeling::setMaxDistance(float maxDistance)
{
    m_maxDistance = maxDistance;
}

const std::vector<std::shared_ptr<ConnectedComponent>>& ConnectedComponentLabeling::getComponents() const
{
    return m_components;
}

const cv::Mat& ConnectedComponentLabeling::getLabelMap() const
{
    return m_labelMap;
}

cv::Mat ConnectedComponentLabeling::getColoredLabelMap()
{
    Utils::getColoredLabelMap(m_labelMap, m_coloredLabelMap);
    return m_coloredLabelMap;
}

void ConnectedComponentLabeling::process(const cv::Mat& foreground,
                                         const cv::Mat& pointCloud)
{
    begin();

    // create a new label map
    if (foreground.cols != m_labelMap.cols || foreground.rows != m_labelMap.rows) {
        m_labelMap = cv::Mat(foreground.rows, foreground.cols, CV_32S);
        m_tempMask = cv::Mat(foreground.rows + 2, foreground.cols + 2, CV_8UC1);
    }
    m_labelMap.setTo(0);

    m_components.clear();

    // find connected components until each point has been labelled
    unsigned int nextLabel = 1;
    for (int i = 0; i < foreground.cols; i++) {
        for (int j = 0; j < foreground.rows; j++) {
            cv::Point curPoint(i, j);
            const unsigned int& label = m_labelMap.at<unsigned int>(curPoint);
            const float& depth = foreground.at<float>(curPoint);

            if (depth > 0 && label == 0) {
                findConnectedComponents(foreground, pointCloud, curPoint, nextLabel);
                nextLabel++;
            }
        }
    }

    // get nearby components
    //for (size_t i = 0; i < m_components.size(); i++) {
    //    std::shared_ptr<ConnectedComponent>& component = m_components[i];
    //    component->nearbyIds = findNearbyComponents(component->id);
    //}

    end();
}

void ConnectedComponentLabeling::findConnectedComponents(const cv::Mat& foreground,
                                                         const cv::Mat& pointCloud,
                                                         const cv::Point& seed,
                                                         unsigned int label)
{
    m_tempMask.setTo(0);

    int flags = 4 | (255 << 8) | cv::FLOODFILL_MASK_ONLY;
    cv::floodFill(foreground, m_tempMask, seed, cv::Scalar(0), 0, cv::Scalar(m_maxDistance), cv::Scalar(m_maxDistance), flags);

    int size = 0;

    // initialize bounding box
    float bbMinDepth = 1000;
    float bbMaxDepth = 0;
    cv::Point bbMinPoint(foreground.cols, foreground.rows);
    cv::Point bbMaxPoint(0, 0);

    cv::Point3f bbMinPoint3d(1000, 1000, 1000);
    cv::Point3f bbMaxPoint3d(-1000, -1000, -1);

    float m10 = 0, m01 = 0;

    for (int i = 1; i < m_tempMask.rows - 1; i++) {
        const uchar* maskRow = m_tempMask.ptr<uchar>(i);
        unsigned int* labelRow = m_labelMap.ptr<unsigned int>(i - 1);
        const float* foregroundRow = foreground.ptr<float>(i - 1);
        const cv::Vec3f* pointsRow = pointCloud.ptr<cv::Vec3f>(i - 1);

        for (int j = 1; j < m_tempMask.cols - 1; j++) {
            uchar maskVal = maskRow[j];
            float foregroundVal = foregroundRow[j - 1];
            const cv::Vec3f& pointVal = pointsRow[j - 1];

            if (maskVal > 0) {
                labelRow[j - 1] = label;

                // update 2d bounding box
                {
                    if (foregroundVal < bbMinDepth)
                        bbMinDepth = foregroundVal;
                    if (foregroundVal > bbMaxDepth)
                        bbMaxDepth = foregroundVal;

                    if (j - 1 < bbMinPoint.x)
                        bbMinPoint.x = j - 1;
                    if (j - 1 > bbMaxPoint.x)
                        bbMaxPoint.x = j - 1;

                    if (i - 1 < bbMinPoint.y)
                        bbMinPoint.y = i - 1;
                    if (i - 1 > bbMaxPoint.y)
                        bbMaxPoint.y = i - 1;
                }

                // update 3d bounding box
                {
                    if (pointVal[0] < bbMinPoint3d.x)
                        bbMinPoint3d.x = pointVal[0];
                    if (pointVal[0] > bbMaxPoint3d.x)
                        bbMaxPoint3d.x = pointVal[0];

                    if (pointVal[1] < bbMinPoint3d.y)
                        bbMinPoint3d.y = pointVal[1];
                    if (pointVal[1] > bbMaxPoint3d.y)
                        bbMaxPoint3d.y = pointVal[1];

                    if (pointVal[2] < bbMinPoint3d.z)
                        bbMinPoint3d.z = pointVal[2];
                    if (pointVal[2] > bbMaxPoint3d.z)
                        bbMaxPoint3d.z = pointVal[2];
                }

                m10 += j - 1;
                m01 += i - 1;

                size++;
            }
        }
    }

    if (size > 1) {
        // create a new component
        std::shared_ptr<ConnectedComponent> component(new ConnectedComponent());
        component->id = label;
        component->area = size;
        component->boundingBox2d = BoundingBox2D(bbMinPoint, bbMaxPoint, bbMinDepth, bbMaxDepth);
        component->boundingBox3d = BoundingBox3D(bbMinPoint3d, bbMaxPoint3d);

        // compute center of mass
        component->centerOfMass = cv::Point2f((float)(m10 / size), (float)(m01 / size));
        component->centerDepth = foreground.at<float>(cv::Point((int)component->centerOfMass.x, (int)component->centerOfMass.y));

        m_components.push_back(component);
    }
}
}
