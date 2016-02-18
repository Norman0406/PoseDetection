#include "tracking.h"
#include "connectedcomponentlabeling.h"
#include <utils/utils.h>

#include <algorithm>

namespace pose
{
Tracking::Tracking()
    : Module("Tracking")
{
    setSearchRadius(0.1f);
    m_minBoundingBoxOverlap = 0.5f;
}

Tracking::~Tracking()
{
}

void Tracking::setSearchRadius(float radius)
{
    m_searchRadius = radius;
}

const std::vector<std::shared_ptr<TrackingCluster>>& Tracking::getClusters() const
{
    return m_trackingClusters;
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

void Tracking::process(const cv::Mat& foreground,
                       const cv::Mat& labelMap,
                       const std::vector<std::shared_ptr<ConnectedComponent>>& components,
                       const cv::Mat& projectionMatrix)
{
    begin();

    UNUSED(projectionMatrix);

    // create a new label map
    if (foreground.cols != m_labelMap.cols || foreground.rows != m_labelMap.rows) {
        m_labelMap = cv::Mat(foreground.rows, foreground.cols, CV_32S);
    }
    m_labelMap.setTo(0);

    /*createAssignments(components);
    //cluster();
    cluster2();
    //resolveSplits();
    deleteLostObjects();
    createLabelMap(labelMap);*/

    if (m_trackingClusters.empty()) {
        std::shared_ptr<TrackingCluster> cluster(new TrackingCluster());
        cluster->id = 1;
        cluster->frames = 1;
        m_trackingClusters.push_back(cluster);
    }

    for (int i = 0; i < foreground.rows; i++) {
        for (int j = 0; j < foreground.cols; j++) {
            if (foreground.ptr<float>(i)[j] > 0)
                m_labelMap.ptr<unsigned int>(i)[j] = 1;
        }
    }

    end();
}

void Tracking::createAssignments(const std::vector<std::shared_ptr<ConnectedComponent>>& components)
{
    // This is a very simple but fast tracking algorithm. It is not optimal, since an optimal solution can only
    // be retrieved by solving the assignment problem. However, it seems to be feasible in most ways. Assigning
    // newly detected components to tracked objects works by means of a bounding box overlapping criterion and
    // tracking movement is done by comparing the distances of bounding box planes, i.e. the six adjacent planes
    // that define the bounding box. This guarantees that a still standing person that is temporally overlapped
    // by another person can still be tracked, since one of the bounding box planes will not move while the other
    // will adapt to the overlapping object.

    // see https://github.com/WeatherGod/MHT
    // http://www.polymtl.ca/litiv/doc/TorabiBilodeauCRV2009.pdf    (mht blob tracking)
    // http://kobus.ca/research/resources/doc/doxygen/dir_711d5743d84aa09a37d575a6382e5856.html (MCDMCA)

    std::vector<bool> assignedComponents(components.size());
    assignedComponents.assign(assignedComponents.size(), false);

    // try to find correspondences for already tracked objects
    for (size_t i = 0; i < m_trackingObjects.size(); i++) {
        std::shared_ptr<TrackingObject>& object = m_trackingObjects[i];
        const std::shared_ptr<ConnectedComponent>& objectComponent = object->currentComponent;

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
            //float overlapFactor1 = objectComponent->boundingBox2d.getOverlapFactor(component->boundingBox2d);
            //float overlapFactor2 = component->boundingBox2d.getOverlapFactor(objectComponent->boundingBox2d);

            //if ((overlapFactor1 > m_minBoundingBoxOverlap || overlapFactor2 > m_minBoundingBoxOverlap)) {
            if (true) {
                // bounding boxes overlap at least the minimum value and they are similar in size

                float distance = Utils::distance(objectComponent->boundingBox3d, component->boundingBox3d, m_searchRadius);

                if (distance < minAnchorDist && distance < m_searchRadius) {
                    minAnchorDist = distance;
                    nearestComponent = component;
                    nearestComponentIndex = j;
                }
            }
        }

        if (nearestComponent) {
            // a nearest component has been found for this tracking object
            object->frames++;
            object->state = TrackingObject::TS_ACTIVE;
            object->previousComponent = object->currentComponent;
            object->currentComponent = nearestComponent;
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
            object->id = getNextFreeId(m_trackingObjects);
            object->frames = 1;
            object->state = TrackingObject::TS_ACTIVE;
            object->currentComponent = component;
            m_trackingObjects.push_back(object);
        }
    }
}

void Tracking::cluster()
{
    // find the biggest object
    // find smaller objects that have some overlap
    // assign all objects into the same cluster

    // only cluster objects that are either new and don't belong to any cluster,
    // or that belong to the same cluster

    std::vector<unsigned int> assignedObjects;
    assignedObjects.reserve(m_trackingObjects.size());
    do {
        // find biggest object
        std::shared_ptr<TrackingObject> biggestObject;
        for (size_t i = 0; i < m_trackingObjects.size(); i++) {
            std::shared_ptr<TrackingObject>& object = m_trackingObjects[i];

            bool hasBeenAssigned = std::find(assignedObjects.begin(), assignedObjects.end(), object->id) != assignedObjects.end();
            if (!hasBeenAssigned) {
                if (!biggestObject || object->currentComponent->area > biggestObject->currentComponent->area)
                    biggestObject = object;
            }
        }

        if (biggestObject) {
            // find or create the associated cluster
            std::shared_ptr<TrackingCluster> cluster;
            if (biggestObject->assignedCluster) {
                cluster = biggestObject->assignedCluster;
                cluster->frames++;
            }
            else {
                cluster = std::shared_ptr<TrackingCluster>(new TrackingCluster());
                cluster->id = getNextFreeId(m_trackingClusters);
                cluster->frames = 1;
                cluster->clusterObjects.push_back(biggestObject);
                biggestObject->assignedCluster = cluster;
                m_trackingClusters.push_back(cluster);
            }

            // find smaller objects and merge them into the same cluster
            for (size_t i = 0; i < m_trackingObjects.size(); i++) {
                std::shared_ptr<TrackingObject>& object = m_trackingObjects[i];

                bool hasBeenAssigned = std::find(assignedObjects.begin(), assignedObjects.end(), object->id) != assignedObjects.end();
                if (object == biggestObject || hasBeenAssigned)
                    continue;

                // object can only be assigned if has either not been assigned before or if the cluster id is the same
                //bool objectCanBeAssigned = !object->assignedCluster || object->assignedCluster->id == cluster->id;

                // compute overlap and assign to cluster
                float factor = 0;
                //float overlapFactor = biggestObject->currentComponent->boundingBox2d.getOverlapFactor(object->currentComponent->boundingBox2d);
                float overlapFactor = object->currentComponent->boundingBox2d.getOverlapFactor(biggestObject->currentComponent->boundingBox2d);
                if (overlapFactor > factor) {
                    bool isContained = std::find(cluster->clusterObjects.begin(), cluster->clusterObjects.end(), object) != cluster->clusterObjects.end();

                    if (!isContained) {
                        cluster->clusterObjects.push_back(object);
                        object->assignedCluster = cluster;
                    }
                }
                /*else
                    object->assignedCluster.reset();*/
            }

            for (size_t i = 0; i < cluster->clusterObjects.size(); i++)
                assignedObjects.push_back(cluster->clusterObjects[i]->id);
        }
    } while (assignedObjects.size() < m_trackingObjects.size());
}

void Tracking::cluster2()
{
    // release previous cluster assignments if bounding boxes do not overlap anymore
    for (size_t i = 0; i < m_trackingObjects.size(); i++) {
        std::shared_ptr<TrackingObject>& object1 = m_trackingObjects[i];

        for (size_t j = 0; j < m_trackingObjects.size(); j++) {
            std::shared_ptr<TrackingObject>& object2 = m_trackingObjects[j];

            if (object1 == object2 || object1->assignedCluster != object2->assignedCluster)
                continue;

            // mark the compared object as not tracked
            if (!object1->currentComponent->boundingBox2d.overlapsWith(object2->currentComponent->boundingBox2d)) {
                object2->assignedCluster.reset();
            }
        }
    }

    // remove objects from clusters that are not tracked anymore
    for (size_t i = 0; i < m_trackingClusters.size(); i++) {
        std::shared_ptr<TrackingCluster>& cluster = m_trackingClusters[i];

        for (auto it = cluster->clusterObjects.begin(); it != cluster->clusterObjects.end();) {
            const std::shared_ptr<TrackingObject>& object = *it;
            if (!object->assignedCluster)
                it = cluster->clusterObjects.erase(it);
            else
                it++;
        }
    }

    // create new cluster assignments for overlapping bounding boxes
    for (size_t i = 0; i < m_trackingObjects.size(); i++) {
        std::shared_ptr<TrackingObject>& object1 = m_trackingObjects[i];

        /*std::shared_ptr<TrackingCluster> cluster;
        if (!object1->assignedCluster) {
            cluster = std::shared_ptr<TrackingCluster>(new TrackingCluster());
            cluster->id = getNextFreeId(m_trackingClusters);
            cluster->frames = 1;
            cluster->clusterObjects.push_back(object1);
            object1->assignedCluster = cluster;
            m_trackingClusters.push_back(cluster);
        }
        else {
            cluster = object1->assignedCluster;
            cluster->frames++;
        }*/

        for (size_t j = 0; j < m_trackingObjects.size(); j++) {
            std::shared_ptr<TrackingObject>& object2 = m_trackingObjects[j];

            /*if (object1 == object2 || object2->assignedCluster != 0)
                continue;

            if (object1->currentComponent->boundingBox2d.overlapsWith(object2->currentComponent->boundingBox2d)) {
                bool isContained = std::find(cluster->clusterObjects.begin(), cluster->clusterObjects.end(), object2) != cluster->clusterObjects.end();
                if (!isContained)
                    cluster->clusterObjects.push_back(object2);
                object2->assignedCluster = cluster;
            }*/

            if (object1 == object2)// || object2->assignedCluster != 0)
                continue;

            if (object1->currentComponent->boundingBox2d.overlapsWith(object2->currentComponent->boundingBox2d)) {

                std::shared_ptr<TrackingCluster> cluster;
                if (object1->assignedCluster) {

                }
                else if (object2->assignedCluster) {

                }
                else {
                    cluster = std::shared_ptr<TrackingCluster>(new TrackingCluster());
                    cluster->id = getNextFreeId(m_trackingClusters);
                    cluster->frames = 1;
                    cluster->clusterObjects.push_back(object1);
                    cluster->clusterObjects.push_back(object2);
                    object1->assignedCluster = cluster;
                    m_trackingClusters.push_back(cluster);
                }



                if (!object1->assignedCluster) {
                    cluster = std::shared_ptr<TrackingCluster>(new TrackingCluster());
                    cluster->id = getNextFreeId(m_trackingClusters);
                    cluster->frames = 1;
                    cluster->clusterObjects.push_back(object1);
                    object1->assignedCluster = cluster;
                    m_trackingClusters.push_back(cluster);
                }
                else {
                    cluster = object1->assignedCluster;
                    cluster->frames++;
                }

                /*bool isContained = std::find(cluster->clusterObjects.begin(), cluster->clusterObjects.end(), object2) != cluster->clusterObjects.end();
                if (!isContained)
                    cluster->clusterObjects.push_back(object2);
                object2->assignedCluster = cluster;*/
            }
        }
    }
}

void Tracking::resolveSplits()
{
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
}

void Tracking::deleteLostObjects()
{
    // clean up tracking clusters
    for (auto it1 = m_trackingClusters.begin(); it1 != m_trackingClusters.end();) {
        const std::shared_ptr<TrackingCluster>& cluster = *it1;

        // remove objects that are not tracked anymore
        for (auto it2 = cluster->clusterObjects.begin(); it2 != cluster->clusterObjects.end();) {
            if ((*it2)->state == TrackingObject::TS_LOST)
                it2 = cluster->clusterObjects.erase(it2);
            else
                it2++;
        }

        cluster->update();

        // remove an empty cluster
        if (cluster->clusterObjects.size() == 0)
            it1 = m_trackingClusters.erase(it1);
        else
            it1++;
    }

    // delete lost tracking objects
    for (auto it = m_trackingObjects.begin(); it != m_trackingObjects.end();) {
        if ((*it)->state == TrackingObject::TS_LOST) {
            it = m_trackingObjects.erase(it);
        }
        else
            it++;
    }
}

void Tracking::createLabelMap(const cv::Mat& labelMap)
{
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

                if (object->currentComponent->id == label && object->assignedCluster) {
                    trackingLabel = object->assignedCluster->id;
                    //trackingLabel = object->id;
                    temp.at<cv::Vec3b>(curPoint) = cv::Vec3b(255, 255, 255);
                    break;
                }
            }
        }
    }

    // get nearby components
    for (size_t i = 0; i < m_trackingClusters.size(); i++) {
        const std::shared_ptr<TrackingCluster>& cluster = m_trackingClusters[i];
        // draw the cluster box red
        cv::rectangle(temp, cluster->boundingBox2d.getMinPoint() + cv::Point(-2, -2),
                      cluster->boundingBox2d.getMaxPoint() + cv::Point(2, 2), cv::Scalar(0, 0, 255));

        // draw each cluster entry green
        for (size_t j = 0; j < cluster->clusterObjects.size(); j++) {
            const std::shared_ptr<TrackingObject>& object = cluster->clusterObjects[j];
            cv::rectangle(temp, object->currentComponent->boundingBox2d.getMinPoint(),
                          object->currentComponent->boundingBox2d.getMaxPoint(), cv::Scalar(0, 255, 0));
        }
    }

    cv::imshow("Temp", temp);
}
}
