#include "depthcamerakinectsdk.h"

namespace pose
{
DepthCameraKinectSDK::DepthCameraKinectSDK()
    : m_sensor(0), m_processThread(0), m_terminate(false)
{
}

DepthCameraKinectSDK::~DepthCameraKinectSDK()
{
    close();
}

bool DepthCameraKinectSDK::open()
{
    NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH);

    if (FAILED(NuiCreateSensorByIndex(0, &m_sensor)))
        return false;

    m_nextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

    // Open a depth image stream to receive depth frames
    if (FAILED(m_sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, NUI_IMAGE_RESOLUTION_640x480,
                                            0, 2, m_nextDepthFrameEvent, &m_depthStreamHandle))) {
        return false;
    }

    DWORD width = 0, height = 0;
    NuiImageResolutionToSize(NUI_IMAGE_RESOLUTION_640x480, width, height);

    m_depthMap = cv::Mat(height, width, CV_32F);
    m_depthMapBuffer = cv::Mat(height, width, CV_32F);
    m_depthMapReady = false;

    m_pointCloud = cv::Mat(height, width, CV_32FC3);
    m_pointCloudBuffer = cv::Mat(height, width, CV_32FC3);

    m_terminate = false;
    m_processThread = new std::thread(std::bind(&DepthCameraKinectSDK::processDepth, this));

    return true;
}

void DepthCameraKinectSDK::close()
{
    if (m_processThread) {
        m_terminate = true;
        m_processThread->join();
        delete m_processThread;
    }

    m_sensor->Release();
}

void DepthCameraKinectSDK::processDepth()
{
    while (!m_terminate) {
        if (WaitForSingleObject(m_nextDepthFrameEvent, INFINITE) == WAIT_OBJECT_0) {
            // process depth

            NUI_IMAGE_FRAME frame;
            m_sensor->NuiImageStreamGetNextFrame(m_depthStreamHandle, 0, &frame);

            INuiFrameTexture* texture;
            m_sensor->NuiImageFrameGetDepthImagePixelFrameTexture(m_depthStreamHandle, &frame, nullptr, &texture);

            NUI_LOCKED_RECT rect;
            texture->LockRect(0, &rect, nullptr, 0);

            NUI_DEPTH_IMAGE_PIXEL* bufferRun = reinterpret_cast<NUI_DEPTH_IMAGE_PIXEL*>(rect.pBits);

            std::unique_lock<std::mutex> lock(m_depthMapMutex);

            m_depthMapBuffer.setTo(0);
            m_pointCloudBuffer.setTo(0);

            for (int i = 0; i < m_depthMapBuffer.rows; i++) {
                float* row = m_depthMapBuffer.ptr<float>(i);
                for (int j = 0; j < m_depthMapBuffer.cols; j++) {
                    NUI_DEPTH_IMAGE_PIXEL depthPixel = *bufferRun++;
                    row[j] = (float)(depthPixel.depth / 1000.0f);

                    // TODO: get point cloud
                }
            }

            m_depthMapReady = true;
            m_depthMapReadyCond.notify_all();
            lock.unlock();

            texture->UnlockRect(0);
            m_sensor->NuiImageStreamReleaseFrame(m_depthStreamHandle, &frame);
        }
    }
}

void DepthCameraKinectSDK::iWaitForData()
{
    std::unique_lock<std::mutex> lock(m_depthMapMutex);
    while (!m_depthMapReady)
        m_depthMapReadyCond.wait(lock);

    m_depthMapBuffer.copyTo(m_depthMap);
    m_pointCloudBuffer.copyTo(m_pointCloud);

    m_depthMapReady = false;
}
}
