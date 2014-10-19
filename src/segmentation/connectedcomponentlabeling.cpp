#include "connectedcomponentlabeling.h"
#include <queue>

ConnectedComponentLabeling::ConnectedComponentLabeling()
    : m_minRatio(320), m_maxDistance(0.1f)
{
    setMinRatio(320);
    setMaxDistance(0.1f);
}

ConnectedComponentLabeling::~ConnectedComponentLabeling()
{
}

void ConnectedComponentLabeling::setMinRatio(int minRatio)
{
    m_minRatio = minRatio;
}

void ConnectedComponentLabeling::setMaxDistance(float maxDistance)
{
    m_maxDistance = maxDistance;
}

const cv::Mat& ConnectedComponentLabeling::getLabelMap() const
{
    return m_labelMap;
}

cv::Mat ConnectedComponentLabeling::getColoredLabelMap() const
{
    // TODO: assign each label a different color, starting with the biggest clusters

    cv::Mat coloredLabelMap(m_labelMap.rows, m_labelMap.cols, CV_8UC3);
    coloredLabelMap.setTo(0);

    for (int i = 0; i < m_labelMap.cols; i++) {
        for (int j = 0; j < m_labelMap.rows; j++) {
            const unsigned int& label = m_labelMap.at<unsigned int>(cv::Point(i, j));

            if (label > 0) {
                srand(label);
                cv::Vec3b col((rand() / (float)RAND_MAX) * 255, (rand() / (float)RAND_MAX) * 255, (rand() / (float)RAND_MAX) * 255);
                coloredLabelMap.at<cv::Vec3b>(cv::Point(i, j)) = col;
            }
        }
    }

    return coloredLabelMap;
}

void ConnectedComponentLabeling::process(const cv::Mat& depthMap)
{
    // create a new label map
    if (depthMap.cols != m_labelMap.cols || depthMap.rows != m_labelMap.rows) {
        m_labelMap = cv::Mat(depthMap.rows, depthMap.cols, CV_32S);
        m_minSize = depthMap.cols * depthMap.rows / m_minRatio;
    }
    m_labelMap.setTo(0);

    m_labelsToRemove.clear();

    // find connected components until each point has been labelled
    cv::Point seed(0, 0);
    unsigned int label = 1;
    while (getNextSeed(depthMap, seed)) {
        findConnectedComponents(depthMap, seed, label);
        label++;
    };

    // filter out components that are too small
    filterComponents();
}

bool ConnectedComponentLabeling::getNextSeed(const cv::Mat& depthMap, cv::Point& seed)
{
    // TODO: integrate more strategies
    //return getClosestUnlabelledPoint(depthMap, seed);
    return getNextUnlabelledPoint(depthMap, seed);
}

bool ConnectedComponentLabeling::getNextUnlabelledPoint(const cv::Mat& depthMap, cv::Point& nextPoint)
{
    for (int i = 0; i < depthMap.cols; i++) {
        for (int j = 0; j < depthMap.rows; j++) {
            cv::Point curPoint(i, j);
            const unsigned int& label = m_labelMap.at<unsigned int>(curPoint);
            const float& depth = depthMap.at<float>(curPoint);

            if (depth > 0 && label == 0) {
                nextPoint = curPoint;
                return true;
            }
        }
    }

    return false;
}

bool ConnectedComponentLabeling::getClosestUnlabelledPoint(const cv::Mat& depthMap, cv::Point& closestPoint)
{
    float minDepth = std::numeric_limits<float>::max();
    bool found = false;

    for (int i = 0; i < depthMap.cols; i++) {
        for (int j = 0; j < depthMap.rows; j++) {
            cv::Point curPoint(i, j);
            const unsigned int& label = m_labelMap.at<unsigned int>(curPoint);
            const float& depth = depthMap.at<float>(curPoint);

            if (depth > 0 && depth < minDepth && label == 0) {
                minDepth = depth;
                closestPoint = curPoint;
                found = true;
            }
        }
    }

    return found;
}

void ConnectedComponentLabeling::findConnectedComponents(const cv::Mat& depthMap, const cv::Point& seed, unsigned int label)
{
    std::queue<cv::Point> queue;
    queue.push(seed);
    m_labelMap.at<unsigned int>(seed) = label;

    int size = 1;

    // The connected component is first written to a temporary label map that is added to the final label map only
    // if the component is big enough. Otherwise, the pixels are marked as IL_DISCARDED.
    do {
        const cv::Point& curPoint = queue.front();
        const float& curPointDepth = depthMap.at<float>(curPoint);
        queue.pop();

        // push valid 8-neighborhood to queue
        for (int k = -1; k <= 1; k++) {
            for (int l = -1; l <= 1; l++) {
                // skip the center pixel
                if (k == 0 && l == 0)
                    continue;

                int iInd = curPoint.x + k;
                int jInd = curPoint.y + l;

                if (iInd < 0 || iInd >= depthMap.cols ||
                    jInd < 0 || jInd >= depthMap.rows)
                    continue;

                cv::Point neighborPoint(iInd, jInd);

                const float& dist = depthMap.at<float>(neighborPoint);
                unsigned int& curLabel = m_labelMap.at<unsigned int>(neighborPoint);

                float distDist = fabs(dist - curPointDepth);

                if (dist > 0 && distDist < m_maxDistance && curLabel == 0) {
                    queue.push(neighborPoint);

                    // label the current point
                    curLabel = label;
                    size++;
                }
            }
        }
    } while (!queue.empty());

    // connected component is too small, mark it as to be removed
    if (size < m_minSize)
        m_labelsToRemove.push_back(label);
}

void ConnectedComponentLabeling::filterComponents()
{
    for (int i = 0; i < m_labelMap.cols; i++) {
        for (int j = 0; j < m_labelMap.rows; j++) {
            unsigned int& curLabel = m_labelMap.at<unsigned int>(cv::Point(i, j));

            // remove the label
            if (std::find(m_labelsToRemove.begin(), m_labelsToRemove.end(), curLabel) != m_labelsToRemove.end())
                curLabel = 0;
        }
    }
}
