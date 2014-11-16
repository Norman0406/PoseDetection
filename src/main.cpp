#include <iostream>
#include <opencv2/opencv.hpp>

#include <input/depthcamerakinectsdk2.h>
#include <segmentation/connectedcomponentlabeling.h>
#include <segmentation/staticmap.h>
#include <segmentation/tracking.h>

using namespace std;

void main()
{
    pose::DepthCamera* camera = new pose::DepthCameraKinectSDK2();
    camera->open();

    pose::ConnectedComponentLabeling* ccLabelling = new pose::ConnectedComponentLabeling();
    pose::StaticMap* staticMap = new pose::StaticMap();
    pose::Tracking* tracking = new pose::Tracking();

    do {
        camera->waitForData();

        if (camera->ready()) {
            // get depth map
            const cv::Mat& depthMap = camera->getDepthMap();
            const cv::Mat& pointCloud = camera->getPointCloud();
            const cv::Mat& projectionMatrix = camera->getProjectionMatrix();

            cv::imshow("Depth", depthMap * 0.2f);

            // compute static background
            staticMap->process(depthMap);
            //cv::imshow("Background", staticMap->getBackground() * 0.2f);
            //cv::imshow("Foreground", staticMap->getForeground() * 0.2f);

            // compute connected components
            ccLabelling->process(staticMap->getForeground(), pointCloud);
            cv::imshow("Labels", ccLabelling->getColoredLabelMap());

            // track connected components
            tracking->process(depthMap, ccLabelling->getLabelMap(), ccLabelling->getComponents(), projectionMatrix);
            cv::imshow("Tracking Labels", tracking->getColoredLabelMap());
        }


    } while (cv::waitKey(1) != 27);

    camera->close();

    delete camera;
    delete ccLabelling;
    delete staticMap;
    delete tracking;
}
