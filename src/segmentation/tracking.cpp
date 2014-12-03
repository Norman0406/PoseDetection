#include "tracking.h"
#include "connectedcomponentlabeling.h"
#include <utils/utils.h>

#include <algorithm>

namespace pose
{
Tracking::Tracking()
{
    setSearchRadius(0.2f);
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
            object->currentComponent = std::shared_ptr<MergedComponent>(new MergedComponent(components[i]));
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
        const std::shared_ptr<MergedComponent>& objectComponent = object->currentComponent;

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

            //if ((overlapFactor1 > m_minBoundingBoxOverlap || overlapFactor2 > m_minBoundingBoxOverlap)) {
            if (true) {
                // bounding boxes overlap at least the minimum value and they are similar in size

                float distance = Utils::distance(objectComponent->boundingBox3d, component->boundingBox3d, m_searchRadius);

                if (distance < minAnchorDist && distance < m_searchRadius) {
                    minAnchorDist = distance;
                    nearestComponent = component;
                    nearestComponentIndex = j;
                }

                // compute the distance between left and right bounding box anchor lines
                /*float distLeft = objectComponent->boundingBox3d.getAnchorDistance(BoundingBox3D::AT_LEFT, component->boundingBox3d);
                float distRight = objectComponent->boundingBox3d.getAnchorDistance(BoundingBox3D::AT_RIGHT, component->boundingBox3d);
                float distTop = objectComponent->boundingBox3d.getAnchorDistance(BoundingBox3D::AT_TOP, component->boundingBox3d);
                float distBottom = objectComponent->boundingBox3d.getAnchorDistance(BoundingBox3D::AT_BOTTOM, component->boundingBox3d);

                float d1 = sqrtf(distLeft * distLeft + distTop * distTop);
                float d2 = sqrtf(distLeft * distLeft + distBottom * distBottom);
                float d3 = sqrtf(distRight * distRight + distTop * distTop);
                float d4 = sqrtf(distRight * distRight + distBottom * distBottom);

                if (d1 < m_searchRadius || d2 < m_searchRadius || d3 < m_searchRadius || d4 < m_searchRadius) {
                    if (d1 < minAnchorDist) {
                        minAnchorDist = d1;
                        nearestComponent = component;
                        nearestComponentIndex = j;
                    }
                    else if (d2 < minAnchorDist) {
                        minAnchorDist = d2;
                        nearestComponent = component;
                        nearestComponentIndex = j;
                    }
                    else if (d3 < minAnchorDist) {
                        minAnchorDist = d3;
                        nearestComponent = component;
                        nearestComponentIndex = j;
                    }
                    else if (d4 < minAnchorDist) {
                        minAnchorDist = d4;
                        nearestComponent = component;
                        nearestComponentIndex = j;
                    }
                }*/

                // either one of them has to be similar
                /*if (distLeft < m_searchRadius || distRight < m_searchRadius) {
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
                }*/
            }
        }

        if (nearestComponent) {
            // a nearest component has been found for this tracking object
            object->frames++;
            object->state = TrackingObject::TS_ACTIVE;
            object->previousComponent = object->currentComponent;
            object->currentComponent = std::shared_ptr<MergedComponent>(new MergedComponent(nearestComponent));
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
            object->currentComponent = std::shared_ptr<MergedComponent>(new MergedComponent(component));
            m_trackingObjects.push_back(object);
        }
    }

    // resolve splits: multiple components are new that share the same bounding box of one previously tracked object

    /*for (size_t i = 0; i < m_trackingObjects.size(); i++) {
        std::shared_ptr<TrackingObject>& prevObject = m_trackingObjects[i];

        // take older objects
        if (prevObject->frames > 1 && prevObject->state == TrackingObject::TS_ACTIVE) {
            std::vector<std::shared_ptr<TrackingObject>> objectsToMerge;

            for (size_t j = 0; j < m_trackingObjects.size(); j++) {
                std::shared_ptr<TrackingObject>& newObject = m_trackingObjects[j];

                // don't compare the same object
                if (i == j)
                    continue;

                // and look for new objects and add if they almost completely overlap the previous bounding box
                if (newObject->frames == 1 && newObject->state == TrackingObject::TS_ACTIVE) {
                    // TODO: use size overlap as a second measure
                    // a low first overlap factor and a higher second overlap factor

                    float overlapFactor1 = newObject->currentComponent->boundingBox2d.getOverlapFactor(prevObject->previousComponent->boundingBox2d);
                    //float overlapFactor2 = prevObject->previousComponent->boundingBox2d.getOverlapFactor(newObject->currentComponent->boundingBox2d);
                    if (overlapFactor1 >= 0.8)// || overlapFactor2 > 0.8)
                        objectsToMerge.push_back(newObject);
                }
            }

            // create a new merged component and remove the new components
            for (size_t j = 0; j < objectsToMerge.size(); j++) {
                std::shared_ptr<TrackingObject> object = objectsToMerge[j];
                const std::shared_ptr<MergedComponent>& component = object->currentComponent;

                // merge all subcomponents with the current component
                for (size_t k = 0; k < component->mergedComponents.size(); k++)
                    prevObject->currentComponent->mergedComponents.push_back(component->mergedComponents[k]);
                prevObject->currentComponent->update();

                // mark the object as lost so it is deleted later
                object->state = TrackingObject::TS_LOST;
            }
        }
    }*/

    // TODO: merging is only valid as long as the components are not too far away from each other. In this case, detect a split

    // check if the merged components still overlap and split if they don't
    // maybe also check if the components are far away from each other

    /*int size = m_trackingObjects.size();
    for (size_t i = 0; i < size; i++) {
        std::shared_ptr<TrackingObject>& object = m_trackingObjects[i];

        if (object->frames > 1 && object->state == TrackingObject::TS_ACTIVE) {
            std::vector<std::shared_ptr<ConnectedComponent>> objectsToSplit;

            for (auto it1 = object->currentComponent->mergedComponents.begin(); it1 != object->currentComponent->mergedComponents.end(); it1++) {
                const std::shared_ptr<ConnectedComponent>& component1 = *it1;

                for (auto it2 = object->currentComponent->mergedComponents.begin(); it2 != object->currentComponent->mergedComponents.end(); it2++) {
                    if (it1 == it2)
                        continue;

                    const std::shared_ptr<ConnectedComponent>& component2 = *it2;

                    float overlapFactor = component1->boundingBox2d.getOverlapFactor(component2->boundingBox2d);
                    if (overlapFactor < 0.1) {
                        objectsToSplit.push_back(component2);
                    }
                }
            }

            // create a new trajectory for split objects
            for (size_t j = 0; j < objectsToSplit.size(); j++) {
                const std::shared_ptr<ConnectedComponent>& component = objectsToSplit[j];

                // create a new tracking object for the split component
                std::shared_ptr<TrackingObject> object(new TrackingObject());
                object->id = getNextFreeId();
                object->frames = 1;
                object->state = TrackingObject::TS_ACTIVE;
                object->currentComponent = std::shared_ptr<MergedComponent>(new MergedComponent(component));
                m_trackingObjects.push_back(object);

                // remove the split component from the list of merged components
                auto it = std::find(object->currentComponent->mergedComponents.begin(),
                                    object->currentComponent->mergedComponents.end(),
                                    component);
                object->currentComponent->mergedComponents.erase(it);
                object->currentComponent->update();
            }
        }
    }*/




    // special split case: multiple new components that do not overlap one another share the same bounding box of one previously tracked objects (separating people)

    // resolve merges: several separate tracked objects suddenly appear in one combined bounding box

    // special merge case: several objects are merged whose bounding boxes do not overlap one another (touching people)

    // delete lost objects
    for (auto it = m_trackingObjects.begin(); it != m_trackingObjects.end();) {
        if ((*it)->state == TrackingObject::TS_LOST) {
            it = m_trackingObjects.erase(it);
        }
        else
            it++;
    }

    cv::Mat temp(labelMap.rows, labelMap.cols, CV_8UC3);
    temp.setTo(0);

    // create a correctly labelled tracking image
    for (int i = 0; i < labelMap.cols; i++) {
        for (int j = 0; j < labelMap.rows; j++) {
            cv::Point curPoint(i, j);
            const unsigned int& label = labelMap.at<unsigned int>(curPoint);
            unsigned int& trackingLabel = m_labelMap.at<unsigned int>(curPoint);

            // find the tracking id for this label
            for (size_t k = 0; k < m_trackingObjects.size(); k++) {
                const std::shared_ptr<TrackingObject>& object = m_trackingObjects[k];

                // label all subcomponents with the same id
                for (size_t l = 0; l < object->currentComponent->mergedComponents.size(); l++) {
                    if (object->currentComponent->mergedComponents[l]->id == label) {
                        trackingLabel = object->id;
                        temp.at<cv::Vec3b>(curPoint) = cv::Vec3b(255, 255, 255);
                        break;
                    }
                }
            }
        }
    }

    // get nearby components
    for (size_t i = 0; i < m_trackingObjects.size(); i++) {
        std::shared_ptr<TrackingObject>& object = m_trackingObjects[i];
        cv::rectangle(temp, object->currentComponent->boundingBox2d.getMinPoint(), object->currentComponent->boundingBox2d.getMaxPoint(), cv::Scalar(0, 0, 255));

        if (object->currentComponent->mergedComponents.size() > 1) {
            for (size_t j = 0; j < object->currentComponent->mergedComponents.size(); j++) {
                cv::rectangle(temp, object->currentComponent->mergedComponents[j]->boundingBox2d.getMinPoint(),
                              object->currentComponent->mergedComponents[j]->boundingBox2d.getMaxPoint(), cv::Scalar(0, 255, 0));
            }
        }
    }

    cv::imshow("Temp", temp);
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
