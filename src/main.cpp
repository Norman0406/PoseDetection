#include <iostream>
#include <opencv2/opencv.hpp>

#include <input/depthcamerakinectsdk2.h>
#include <segmentation/connectedcomponentlabeling.h>
#include <segmentation/staticmap.h>
#include <segmentation/tracking.h>
#include <utils/numberedfilereader.h>
#include <utils/numberedfilewriter.h>
#include <utils/utils.h>

using namespace std;

void main()
{
    /*pose::DepthCamera* camera = new pose::DepthCameraKinectSDK2();
    camera->open();*/

    pose::ConnectedComponentLabeling* ccLabelling = new pose::ConnectedComponentLabeling();
    pose::StaticMap* staticMap = new pose::StaticMap();
    pose::Tracking* tracking = new pose::Tracking();

    std::string depthFiles("d:/sequences/scene1/depth/depth_%i.cvm");
    std::string pointCloudFiles("d:/sequences/scene1/pointcloud/pointcloud_%i.cvm");
    std::string foregroundFiles("d:/sequences/scene1/foreground/foreground_%i.cvm");
    std::string projectionFile("d:/sequences/scene1/projection.cvm");
    std::string backgroundFile("d:/sequences/scene1/background.cvm");

    pose::NumberedFileWriter writerDepth(depthFiles);
    pose::NumberedFileWriter writerPointCloud(pointCloudFiles);
    pose::NumberedFileWriter writerForeground(foregroundFiles);

    /*const int startFrame = 280;
    const int endFrame = 300;*/
    const int startFrame = 0;
    const int endFrame = -1;
    const bool loop = true;
    pose::NumberedFileReader readerDepth(depthFiles, startFrame, endFrame, loop);
    pose::NumberedFileReader readerPointCloud(pointCloudFiles, startFrame, endFrame, loop);
    pose::NumberedFileReader readerForeground(foregroundFiles, startFrame, endFrame, loop);

    cv::Mat foreground;
    cv::Mat background;
    cv::Mat depthMap;
    cv::Mat pointCloud;
    cv::Mat projectionMatrix;

    pose::Utils::loadCvMat(projectionFile.c_str(), projectionMatrix);
    pose::Utils::loadCvMat(backgroundFile.c_str(), background);

    bool step = false;
    bool paused = false;
    int lastKey = -1;
    do {
        if (lastKey == 'p')
            paused = !paused;
        else if (lastKey == 'n' && paused)
            step = true;
        else if (lastKey == 'f')
            std::cout << "frame " << readerDepth.getFrameIndex() << std::endl;

        if (paused && !step)
            continue;

        if (step)
            step = false;

        /*camera->waitForData();

        if (!camera->ready()) {
            continue;
        }

        writerDepth.write(camera->getDepthMap());
        writerPointCloud.write(camera->getPointCloud());

        camera->getDepthMap().copyTo(depthMap);
        camera->getPointCloud().copyTo(pointCloud);
        if (projectionMatrix.empty()) {
            camera->getProjectionMatrix().copyTo(projectionMatrix);
            pose::Utils::saveCvMat(projectionFile.c_str(), projectionMatrix);
        }*/

        int k = 0;
        if (readerDepth.getFrameIndex() == 138)
            k = 1;

        if (!readerDepth.read(depthMap) || !readerPointCloud.read(pointCloud)) {
            //pose::Utils::saveCvMat(backgroundFile.c_str(), background);
            continue;
        }

        cv::imshow("Depth", depthMap * 0.2f);
        //cv::imshow("Background", background * 0.2f);

        // compute static background
        /*staticMap->process(depthMap);
        cv::imshow("Background", staticMap->getBackground() * 0.2f);
        cv::imshow("Foreground", staticMap->getForeground() * 0.2f);

        writerForeground.write(staticMap->getForeground());
        staticMap->getBackground().copyTo(background);*/

        readerForeground.read(foreground);

        // compute connected components
        ccLabelling->process(foreground, pointCloud);
        cv::imshow("Labels", ccLabelling->getColoredLabelMap());

        // track connected components
        tracking->process(depthMap, ccLabelling->getLabelMap(), ccLabelling->getComponents(), projectionMatrix);
        cv::imshow("Tracking Labels", tracking->getColoredLabelMap());

    } while ((lastKey = cv::waitKey(1)) != 27);

    /*camera->close();
    delete camera;*/
    delete ccLabelling;
    delete staticMap;
    delete tracking;
}
