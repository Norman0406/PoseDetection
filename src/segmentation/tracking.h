#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/opencv.hpp>
#include <memory>
#include "connectedcomponentlabeling.h"
#include <utils/boundingbox2d.h>
#include <utils/boundingbox3d.h>
#include <utils/module.h>

namespace pose
{
struct TrackingCluster;

struct TrackingObject
{
    enum TrackingState {
        TS_ACTIVE,  // the object is currently being tracked
        TS_LOST     // the object could not be found and should be deleted
    };

    unsigned int id;
    int frames;
    TrackingState state;
    std::shared_ptr<TrackingCluster> assignedCluster;
    std::shared_ptr<ConnectedComponent> previousComponent;
    std::shared_ptr<ConnectedComponent> currentComponent;
};

struct TrackingCluster
{
    unsigned int id;
    int frames;
    std::vector<std::shared_ptr<TrackingObject>> clusterObjects;
    BoundingBox2D   boundingBox2d;
    BoundingBox3D   boundingBox3d;

    void update() {
        if (clusterObjects.size() == 1) {
            boundingBox2d = clusterObjects[0]->currentComponent->boundingBox2d;
            boundingBox3d = clusterObjects[0]->currentComponent->boundingBox3d;
        }
        else if (clusterObjects.size() > 1) {
            // initialize bounding box
            float bbMinDepth = 1000;
            float bbMaxDepth = 0;
            cv::Point bbMinPoint(10000, 10000);
            cv::Point bbMaxPoint(0, 0);
            cv::Point3f bbMinPoint3d(1000, 1000, 1000);
            cv::Point3f bbMaxPoint3d(-1000, -1000, -1);

            for (size_t i = 0; i < clusterObjects.size(); i++) {
                const std::shared_ptr<ConnectedComponent>& component = clusterObjects[i]->currentComponent;

                // update 2d bounding box
                {
                    if (component->boundingBox2d.getMinDepth() < bbMinDepth)
                        bbMinDepth = component->boundingBox2d.getMinDepth();
                    if (component->boundingBox2d.getMaxDepth() > bbMaxDepth)
                        bbMaxDepth = component->boundingBox2d.getMaxDepth();

                    if (component->boundingBox2d.getMinPoint().x < bbMinPoint.x)
                        bbMinPoint.x = component->boundingBox2d.getMinPoint().x;
                    if (component->boundingBox2d.getMaxPoint().x > bbMaxPoint.x)
                        bbMaxPoint.x = component->boundingBox2d.getMaxPoint().x;

                    if (component->boundingBox2d.getMinPoint().y < bbMinPoint.y)
                        bbMinPoint.y = component->boundingBox2d.getMinPoint().y;
                    if (component->boundingBox2d.getMaxPoint().y > bbMaxPoint.y)
                        bbMaxPoint.y = component->boundingBox2d.getMaxPoint().y;
                }

                // update 3d bounding box
                {
                    if (component->boundingBox3d.getMinPoint().x < bbMinPoint3d.x)
                        bbMinPoint3d.x = component->boundingBox3d.getMinPoint().x;
                    if (component->boundingBox3d.getMaxPoint().x > bbMaxPoint3d.x)
                        bbMaxPoint3d.x = component->boundingBox3d.getMaxPoint().x;

                    if (component->boundingBox3d.getMinPoint().y< bbMinPoint3d.y)
                        bbMinPoint3d.y = component->boundingBox3d.getMinPoint().y;
                    if (component->boundingBox3d.getMaxPoint().y > bbMaxPoint3d.y)
                        bbMaxPoint3d.y = component->boundingBox3d.getMaxPoint().y;

                    if (component->boundingBox3d.getMinPoint().z < bbMinPoint3d.z)
                        bbMinPoint3d.z = component->boundingBox3d.getMinPoint().z;
                    if (component->boundingBox3d.getMaxPoint().z > bbMaxPoint3d.z)
                        bbMaxPoint3d.z = component->boundingBox3d.getMaxPoint().z;
                }
            }

            boundingBox2d = BoundingBox2D(bbMinPoint, bbMaxPoint, bbMinDepth, bbMaxDepth);
            boundingBox3d = BoundingBox3D(bbMinPoint3d, bbMaxPoint3d);
        }
    }
};

class Tracking
        : public Module
{
public:
    Tracking();
    ~Tracking();

    void setSearchRadius(float);

    const std::vector<std::shared_ptr<TrackingCluster>>& getClusters() const;
    const cv::Mat& getLabelMap() const;
    cv::Mat getColoredLabelMap();

    void process(const cv::Mat& foreground,
                 const cv::Mat& labelMap,
                 const std::vector<std::shared_ptr<ConnectedComponent>>& components,
                 const cv::Mat& projectionMatrix);

private:
    void createAssignments(const std::vector<std::shared_ptr<ConnectedComponent>>& components);
    void cluster();
    void cluster2();
    void resolveSplits();
    void deleteLostObjects();
    void createLabelMap(const cv::Mat& labelMap);

    template <typename T>
    int getNextFreeId(const std::vector<std::shared_ptr<T>>& objects) const;

    cv::Mat m_labelMap;
    cv::Mat m_coloredLabelMap;

    std::vector<std::shared_ptr<TrackingObject>> m_trackingObjects;
    std::vector<std::shared_ptr<TrackingCluster>> m_trackingClusters;
    float m_searchRadius;
    float m_minBoundingBoxOverlap;
};

template <typename T>
int Tracking::getNextFreeId(const std::vector<std::shared_ptr<T>>& objects) const
{
    int id = 1;

    bool idFound = false;
    do {
        idFound = false;
        for (size_t i = 0; i < objects.size(); i++) {
            if (objects[i]->id == id) {
                idFound = true;
                id++;
                break;
            }
        }
    } while (idFound);

    return id;
}
}

#endif // LABELING_H
