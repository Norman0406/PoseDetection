#include "connectedcomponentlabeling.h"
#include <utils/utils.h>
#include <queue>

namespace pose
{
ConnectedComponentLabeling::ConnectedComponentLabeling()
    : m_maxDistance(0.1f)
{
    setMaxDistance(0.1f);
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
    // create a new label map
    if (foreground.cols != m_labelMap.cols || foreground.rows != m_labelMap.rows) {
        m_labelMap = cv::Mat(foreground.rows, foreground.cols, CV_32S);
        m_tempComponent = cv::Mat(foreground.rows, foreground.cols, CV_8UC1);
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
    for (size_t i = 0; i < m_components.size(); i++) {
        std::shared_ptr<ConnectedComponent>& component = m_components[i];
        component->nearbyIds = findNearbyComponents(component->id);
    }
}

void ConnectedComponentLabeling::findConnectedComponents(const cv::Mat& foreground,
                                                         const cv::Mat& pointCloud,
                                                         const cv::Point& seed,
                                                         unsigned int label)
{
    std::queue<cv::Point> queue;
    queue.push(seed);
    m_labelMap.at<unsigned int>(seed) = label;

    int size = 1;

    m_tempComponent.setTo(0);
    m_tempComponent.at<uchar>(seed) = 255;

    // initialize bounding box
    float bbMinDepth = 1000;
    float bbMaxDepth = 0;
    cv::Point bbMinPoint(foreground.cols, foreground.rows);
    cv::Point bbMaxPoint(0, 0);

    cv::Point3f bbMinPoint3d(1000, 1000, 1000);
    cv::Point3f bbMaxPoint3d(-1000, -1000, -1);

    // The connected component is first written to a temporary label map that is added to the final label map only
    // if the component is big enough. Otherwise, the pixels are marked as IL_DISCARDED.
    do {
        const cv::Point& curPoint = queue.front();
        const float& curPointDepth = foreground.at<float>(curPoint);
        queue.pop();

        // push valid 8-neighborhood to queue
        for (int k = -1; k <= 1; k++) {
            for (int l = -1; l <= 1; l++) {
                // skip the center pixel
                if (k == 0 && l == 0)
                    continue;

                int iInd = curPoint.x + k;
                int jInd = curPoint.y + l;

                if (iInd < 0 || iInd >= foreground.cols ||
                    jInd < 0 || jInd >= foreground.rows)
                    continue;

                cv::Point neighborPoint(iInd, jInd);
                const cv::Vec3f& neighborPoint3d = pointCloud.at<cv::Vec3f>(neighborPoint);

                const float& dist = foreground.at<float>(neighborPoint);
                unsigned int& curLabel = m_labelMap.at<unsigned int>(neighborPoint);

                float distDist = fabs(dist - curPointDepth);

                if (dist > 0 && distDist < m_maxDistance && curLabel == 0) {
                    queue.push(neighborPoint);

                    m_tempComponent.at<uchar>(seed) = 255;

                    // update 2d bounding box
                    {
                        if (dist < bbMinDepth)
                            bbMinDepth = dist;
                        if (dist > bbMaxDepth)
                            bbMaxDepth = dist;

                        if (neighborPoint.x < bbMinPoint.x)
                            bbMinPoint.x = neighborPoint.x;
                        if (neighborPoint.x > bbMaxPoint.x)
                            bbMaxPoint.x = neighborPoint.x;

                        if (neighborPoint.y < bbMinPoint.y)
                            bbMinPoint.y = neighborPoint.y;
                        if (neighborPoint.y > bbMaxPoint.y)
                            bbMaxPoint.y = neighborPoint.y;
                    }

                    // update 3d bounding box
                    {
                        if (neighborPoint3d[0] < bbMinPoint3d.x)
                            bbMinPoint3d.x = neighborPoint3d[0];
                        if (neighborPoint3d[0] > bbMaxPoint3d.x)
                            bbMaxPoint3d.x = neighborPoint3d[0];

                        if (neighborPoint3d[1] < bbMinPoint3d.y)
                            bbMinPoint3d.y = neighborPoint3d[1];
                        if (neighborPoint3d[1] > bbMaxPoint3d.y)
                            bbMaxPoint3d.y = neighborPoint3d[1];

                        if (neighborPoint3d[2] < bbMinPoint3d.z)
                            bbMinPoint3d.z = neighborPoint3d[2];
                        if (neighborPoint3d[2] > bbMaxPoint3d.z)
                            bbMaxPoint3d.z = neighborPoint3d[2];
                    }

                    // label the current point
                    curLabel = label;
                    size++;
                }
            }
        }
    } while (!queue.empty());

    if (size > 1) {
        // create a new component
        std::shared_ptr<ConnectedComponent> component(new ConnectedComponent());
        component->id = label;
        component->area = size;
        component->boundingBox2d = BoundingBox2D(bbMinPoint, bbMaxPoint, bbMinDepth, bbMaxDepth);
        component->boundingBox3d = BoundingBox3D(bbMinPoint3d, bbMaxPoint3d);

        // compute center of mass
        cv::Moments moments = cv::moments(m_tempComponent, true);
        component->centerOfMass = cv::Point2f((float)(moments.m10 / moments.m00), (float)(moments.m01 / moments.m00));
        component->centerDepth = foreground.at<float>(cv::Point((int)component->centerOfMass.x, (int)component->centerOfMass.y));

        m_components.push_back(component);
    }
}

std::vector<unsigned int> ConnectedComponentLabeling::findNearbyComponents(unsigned int id)
{
    std::vector<unsigned int> nearbyIds;

    const int nearbyRadius = 5;

    for (int i = 0; i < m_labelMap.cols; i++) {
        for (int j = 0; j < m_labelMap.rows; j++) {
            cv::Point curPoint(i, j);
            unsigned int& curLabel = m_labelMap.at<unsigned int>(curPoint);

            if (curLabel == id) {
                for (int k = -nearbyRadius; k <= nearbyRadius; k++) {
                    for (int l = -nearbyRadius; l <= nearbyRadius; l++) {
                        if (k == 0 && l == 0)
                            continue;

                        int iInd = curPoint.x + k;
                        int jInd = curPoint.y + l;

                        if (iInd < 0 || iInd >= m_labelMap.cols ||
                            jInd < 0 || jInd >= m_labelMap.rows)
                            continue;

                        cv::Point neighborPoint(iInd, jInd);
                        const unsigned int& neighborLabel = m_labelMap.at<unsigned int>(neighborPoint);

                        if (neighborLabel != 0 && neighborLabel != id &&
                                std::find(nearbyIds.begin(), nearbyIds.end(), neighborLabel) != nearbyIds.end()) {
                            nearbyIds.push_back(neighborLabel);
                        }
                    }
                }
            }
        }
    }

    return nearbyIds;
}
}
