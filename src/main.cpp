#include <iostream>
#include <opencv2/opencv.hpp>

#include <input/depthcamerakinectsdk2.h>
#include <segmentation/connectedcomponentlabeling.h>
#include <segmentation/staticmap.h>

using namespace std;

void main()
{
    DepthCamera* camera = new DepthCameraKinectSDK2();
    camera->open();

    cv::namedWindow("Depth");

    ConnectedComponentLabeling* ccLabelling = new ConnectedComponentLabeling();
    StaticMap* staticMap = new StaticMap();

    do {
        camera->waitForData();

        const cv::Mat& depthMap = camera->getDepthMap();
        cv::imshow("Depth", depthMap * 0.2f);

        staticMap->process(depthMap);

        cv::imshow("Background", staticMap->getBackground() * 0.2f);
        cv::imshow("Foreground", staticMap->getForeground() * 0.2f);

        //ccLabelling->process(staticMap->getForeground());

        //cv::imshow("Labels", ccLabelling->getColoredLabelMap());

    } while (cv::waitKey(1) != 27);

    camera->close();

    delete camera;
    delete ccLabelling;
    delete staticMap;
}
