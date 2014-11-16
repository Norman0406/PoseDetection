#include "tracking.h"
#include "connectedcomponentlabeling.h"
#include <utils/utils.h>

#include <algorithm>

namespace pose
{
Tracking::Tracking()
{
    setSearchRadius(0.2f);
    m_minBoundingBoxOverlap = 0.6f;
}

Tracking::~Tracking()
{
}

void Tracking::setSearchRadius(float radius)
{
    m_searchRadius = radius;
}

const cv::Mat& Tracking::getLabelMap() const
{
    return m_labelMap;
}

cv::Mat Tracking::getColoredLabelMap()
{
    Utils::getColoredLabelMap(m_labelMap, m_coloredLabelMap);
    return m_coloredLabelMap;
}

void Tracking::process(const cv::Mat& depthMap,
                       const cv::Mat& labelMap,
                       const std::vector<std::shared_ptr<ConnectedComponent>>& components,
                       const cv::Mat& projectionMatrix)
{
    // create a new label map
    if (depthMap.cols != m_labelMap.cols || depthMap.rows != m_labelMap.rows) {
        m_labelMap = cv::Mat(depthMap.rows, depthMap.cols, CV_32S);
    }
    m_labelMap.setTo(0);

    if (m_trackingObjects.empty()) {
        // no need to do anything
        if (components.empty())
            return;

        // TODO: already regard overlapping object to merge at this point
        // maybe already give in merged components

        // create a new tracking object for each detected component
        for (size_t i = 0; i < components.size(); i++) {
            std::shared_ptr<TrackingObject> object(new TrackingObject());
            object->id = getNextFreeId();
            object->latestComponent = components[i];
            m_trackingObjects.push_back(object);
        }
        return;
    }

    // see https://github.com/WeatherGod/MHT
    // http://www.polymtl.ca/litiv/doc/TorabiBilodeauCRV2009.pdf    (mht blob tracking)
    // http://kobus.ca/research/resources/doc/doxygen/dir_711d5743d84aa09a37d575a6382e5856.html (MCDMCA)

    // find a match for each already existing object
    for (size_t i = 0; i < m_trackingObjects.size(); i++) {
        std::shared_ptr<TrackingObject>& object = m_trackingObjects[i];
        const BoundingBox2D& objectBox = object->latestComponent->boundingBox2d;

        for (size_t j = 0; j < components.size(); j++) {
            const std::shared_ptr<ConnectedComponent>& component = components[j];
            const BoundingBox2D& componentBox = component->boundingBox2d;

            // compute the overlap factor between new component and existing tracking object
            float overlapFactor = objectBox.getOverlapFactor(componentBox);

            if (overlapFactor > m_minBoundingBoxOverlap) {
                // bounding boxes overlap at least the minimum value

                // compute the distance between left and right bounding box anchor lines
                /*float distLeft = getAnchorDistance(objectBox.left(), componentBox.left(), objectBox.avgDepth, componentBox.avgDepth);
                float distRight = getAnchorDistance(objectBox.right(), componentBox.right(), objectBox.avgDepth, componentBox.avgDepth);

                if (distLeft < m_searchRadius || distRight < m_searchRadius) {

                }*/
            }
        }
    }

    // create new objects
    for (size_t i = 0; i < components.size(); i++) {

    }

    /*const float maxDepthDiff = 0.1f;

    // process each row individually at first
    for (int i = 0; i < depthMap.rows; i++) {
        for (int j = 1; j < depthMap.cols; j++) {
            cv::Point point(j, i);
            const float& prevDist = depthMap.at<float>(cv::Point(j - 1, i));
            const float& dist = depthMap.at<float>(point);
            const float& foreground = foregroundMap.at<float>(point);

            const float depthDiff = prevDist - dist;
        }
    }*/
}

int Tracking::getNextFreeId() const
{
    int id = 0;

    bool idFound = false;
    do {
        idFound = false;
        for (size_t i = 0; i < m_trackingObjects.size(); i++) {
            if (m_trackingObjects[i]->id == id) {
                idFound = true;
                id++;
                break;
            }
        }
    } while (idFound);

    return id;
}
}
