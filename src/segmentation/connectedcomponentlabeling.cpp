#include "connectedcomponentlabeling.h"
#include <utils/utils.h>
#include <queue>

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

const std::vector<ConnectedComponent*>& ConnectedComponentLabeling::getComponents() const
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

void ConnectedComponentLabeling::process(const cv::Mat& foreground)
{
    // create a new label map
    if (foreground.cols != m_labelMap.cols || foreground.rows != m_labelMap.rows) {
        m_labelMap = cv::Mat(foreground.rows, foreground.cols, CV_32S);
        temp = cv::Mat(foreground.rows, foreground.cols, CV_8UC3);
        m_tempComponent = cv::Mat(foreground.rows, foreground.cols, CV_8UC1);
    }
    m_labelMap.setTo(0);

    for (size_t i = 0; i < m_components.size(); i++)
        delete m_components[i];
    m_components.clear();

    temp.setTo(0);

    // find connected components until each point has been labelled
    unsigned int nextLabel = 1;
    for (int i = 0; i < foreground.cols; i++) {
        for (int j = 0; j < foreground.rows; j++) {
            cv::Point curPoint(i, j);
            const unsigned int& label = m_labelMap.at<unsigned int>(curPoint);
            const float& depth = foreground.at<float>(curPoint);

            temp.at<cv::Vec3b>(curPoint) = cv::Vec3b(depth * 100, depth * 100, depth * 100);

            if (depth > 0 && label == 0) {
                findConnectedComponents(foreground, curPoint, nextLabel);
                nextLabel++;
            }
        }
    }

    // get nearby components
    for (size_t i = 0; i < m_components.size(); i++) {
        ConnectedComponent* component = m_components[i];
        component->nearbyIds = findNearbyComponents(component->id);

        cv::rectangle(temp, component->boundingBox.minPoint, component->boundingBox.maxPoint, cv::Scalar(0, 0, 255));
    }

    cv::imshow("Temp", temp);
}

void ConnectedComponentLabeling::findConnectedComponents(const cv::Mat& foreground, const cv::Point& seed, unsigned int label)
{
    std::queue<cv::Point> queue;
    queue.push(seed);
    m_labelMap.at<unsigned int>(seed) = label;

    int size = 1;

    m_tempComponent.setTo(0);
    m_tempComponent.at<uchar>(seed) = 255;

    // initialize bounding box
    BoundingBox boundingBox;
    boundingBox.minDepth = 1000;
    boundingBox.maxDepth = 0;
    boundingBox.minPoint = cv::Point(foreground.cols, foreground.rows);
    boundingBox.maxPoint = cv::Point(0, 0);

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

                const float& dist = foreground.at<float>(neighborPoint);
                unsigned int& curLabel = m_labelMap.at<unsigned int>(neighborPoint);

                float distDist = fabs(dist - curPointDepth);

                if (dist > 0 && distDist < m_maxDistance && curLabel == 0) {
                    queue.push(neighborPoint);

                    m_tempComponent.at<uchar>(seed) = 255;

                    // update bounding box
                    if (dist < boundingBox.minDepth)
                        boundingBox.minDepth = dist;
                    else if (dist > boundingBox.maxDepth)
                        boundingBox.maxDepth = dist;

                    if (neighborPoint.x < boundingBox.minPoint.x)
                        boundingBox.minPoint.x = neighborPoint.x;
                    else if (neighborPoint.x > boundingBox.maxPoint.x)
                        boundingBox.maxPoint.x = neighborPoint.x;

                    if (neighborPoint.y < boundingBox.minPoint.y)
                        boundingBox.minPoint.y = neighborPoint.y;
                    else if (neighborPoint.y > boundingBox.maxPoint.y)
                        boundingBox.maxPoint.y = neighborPoint.y;

                    // label the current point
                    curLabel = label;
                    size++;
                }
            }
        }
    } while (!queue.empty());

    // create a new component
    ConnectedComponent* component = new ConnectedComponent();
    component->id = label;
    component->area = size;
    component->boundingBox = boundingBox;
    component->boundingBox.update();

    // compute center of mass
    cv::Moments moments = cv::moments(m_tempComponent, true);
    component->centerOfMass = cv::Point2f((float)(moments.m10 / moments.m00), (float)(moments.m01 / moments.m00));
    component->centerDepth = foreground.at<float>(cv::Point((int)component->centerOfMass.x, (int)component->centerOfMass.y));

    m_components.push_back(component);
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
