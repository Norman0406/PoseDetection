#include "tracking.h"
#include "connectedcomponentlabeling.h"
#include <utils/utils.h>

#include <algorithm>

namespace pose
{
Tracking::Tracking()
{
    setSearchRadius(0.4f);
    m_minBoundingBoxOverlap = 0.5f;
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

    // This is a very simple but fast tracking algorithm. It is not optimal, since an optimal solution can only
    // be retrieved by solving the assignment problem. However, it seems to be feasible in most ways. Assigning
    // newly detected components to tracked objects works by means of a bounding box overlapping criterion and
    // tracking movement is done by comparing the distances of bounding box planes, i.e. the six adjacent planes
    // that define the bounding box. This guarantees that a still standing person that is temporally overlapped
    // by another person can still be tracked, since one of the bounding box planes will not move while the other
    // will adapt to the overlapping object.

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
            object->frames = 1;
            object->state = TrackingObject::TS_ACTIVE;
            object->latestComponent = components[i];
            m_trackingObjects.push_back(object);
        }
        return;
    }

    // see https://github.com/WeatherGod/MHT
    // http://www.polymtl.ca/litiv/doc/TorabiBilodeauCRV2009.pdf    (mht blob tracking)
    // http://kobus.ca/research/resources/doc/doxygen/dir_711d5743d84aa09a37d575a6382e5856.html (MCDMCA)

    std::vector<bool> assignedComponents(components.size());
    assignedComponents.assign(assignedComponents.size(), false);

    // try to find correspondences for already tracked objects
    for (size_t i = 0; i < m_trackingObjects.size(); i++) {
        std::shared_ptr<TrackingObject>& object = m_trackingObjects[i];
        const std::shared_ptr<ConnectedComponent>& objectComponent = object->latestComponent;

        // will store the selected nearest component
        float minAnchorDist = std::numeric_limits<float>::infinity();
        std::shared_ptr<ConnectedComponent> nearestComponent;
        int nearestComponentIndex = -1;

        // go through all detected objects and try to find find one that matches
        for (size_t j = 0; j < components.size(); j++) {
            const std::shared_ptr<ConnectedComponent>& component = components[j];

            // skip components that have already been assigned
            if (assignedComponents[j] == true)
                continue;

            // compute the overlap factor between new component and existing tracking object
            float overlapFactor1 = objectComponent->boundingBox2d.getOverlapFactor(component->boundingBox2d);
            float overlapFactor2 = component->boundingBox2d.getOverlapFactor(objectComponent->boundingBox2d);

            if ((overlapFactor1 > m_minBoundingBoxOverlap || overlapFactor2 > m_minBoundingBoxOverlap)) {
                // bounding boxes overlap at least the minimum value and they are similar in size

                // compute the distance between left and right bounding box anchor lines
                float distLeft = objectComponent->boundingBox3d.getAnchorDistance(BoundingBox3D::AT_LEFT, component->boundingBox3d);
                float distRight = objectComponent->boundingBox3d.getAnchorDistance(BoundingBox3D::AT_RIGHT, component->boundingBox3d);

                // either one of them has to be similar
                if (distLeft < m_searchRadius || distRight < m_searchRadius) {
                    if (distLeft < minAnchorDist) {
                        minAnchorDist = distLeft;
                        nearestComponent = component;
                        nearestComponentIndex = j;
                    }
                    else if (distRight < minAnchorDist) {
                        minAnchorDist = distRight;
                        nearestComponent = component;
                        nearestComponentIndex = j;
                    }
                }
            }
        }

        if (nearestComponent) {
            // a nearest component has been found for this tracking object
            object->frames++;
            object->state = TrackingObject::TS_ACTIVE;
            object->latestComponent = nearestComponent;
            assignedComponents[nearestComponentIndex] = true;
        }
        else {
            // no nearest object has been found, this object should be removed later
            object->state = TrackingObject::TS_LOST;
        }
    }

    // create new objects for detected components that have not been assigned to already tracked objects
    for (size_t i = 0; i < components.size(); i++) {
        const std::shared_ptr<ConnectedComponent>& component = components[i];

        // only create a new trajectory if this component has not already been assigned
        if (assignedComponents[i] == false) {
            std::shared_ptr<TrackingObject> object(new TrackingObject());
            object->id = getNextFreeId();
            object->frames = 1;
            object->state = TrackingObject::TS_ACTIVE;
            object->latestComponent = component;
            m_trackingObjects.push_back(object);
        }
    }

    // delete lost objects
    for (auto it = m_trackingObjects.begin(); it != m_trackingObjects.end();) {
        if ((*it)->state == TrackingObject::TS_LOST) {
            it = m_trackingObjects.erase(it);
        }
        else
            it++;
    }

    // resolve merge: several separate tracked objects suddenly appear in one combined bounding box

    // special merge case: several objects are merged whose bounding boxes do not overlap one another (touching people)

    // resolve splits: multiple components are new that share the same bounding box of one previously tracked object

    // speciel split case:

    // create a correctly labelled tracking image
    for (int i = 0; i < labelMap.cols; i++) {
        for (int j = 0; j < labelMap.rows; j++) {
            cv::Point curPoint(i, j);
            const unsigned int& label = labelMap.at<unsigned int>(curPoint);
            unsigned int& trackingLabel = m_labelMap.at<unsigned int>(curPoint);

            // find the tracking id for this label
            for (size_t i = 0; i < m_trackingObjects.size(); i++) {
                const std::shared_ptr<TrackingObject>& object = m_trackingObjects[i];
                if (object->latestComponent->id == label) {
                    trackingLabel = object->id;
                    break;
                }
            }
        }
    }
}

int Tracking::getNextFreeId() const
{
    int id = 1;

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