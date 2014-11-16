#include "depthcamerakinectsdk2.h"

namespace pose
{
DepthCameraKinectSDK2::DepthCameraKinectSDK2()
    : m_sensor(0), m_depthFrameSource(0), m_depthFrameReader(0)
{
}

DepthCameraKinectSDK2::~DepthCameraKinectSDK2()
{
    close();
}

bool DepthCameraKinectSDK2::open()
{
    if (FAILED(GetDefaultKinectSensor(&m_sensor)))
        return false;

    if (FAILED(m_sensor->Open()))
        return false;

    if (FAILED(m_sensor->get_DepthFrameSource(&m_depthFrameSource)))
        return false;

    if (FAILED(m_depthFrameSource->OpenReader(&m_depthFrameReader)))
        return false;

    UINT16 minReliableDistance, maxReliableDistance;
    m_depthFrameSource->get_DepthMinReliableDistance(&minReliableDistance);
    m_depthFrameSource->get_DepthMaxReliableDistance(&maxReliableDistance);
    m_minReliableDistance = minReliableDistance / 1000.0f;
    m_maxReliableDistance = maxReliableDistance / 1000.0f;

    IFrameDescription* frameDesc;
    if (FAILED(m_depthFrameSource->get_FrameDescription(&frameDesc)))
        return false;

    frameDesc->get_HorizontalFieldOfView(&m_fovHorizontal);
    frameDesc->get_VerticalFieldOfView(&m_fovVertical);
    frameDesc->get_DiagonalFieldOfView(&m_fovDiagonal);

    unsigned int bytesPerPixel;
    frameDesc->get_BytesPerPixel(&bytesPerPixel);

    frameDesc->get_Width(&m_depthSize.width);
    frameDesc->get_Height(&m_depthSize.height);

    m_depthMap = cv::Mat(m_depthSize.height, m_depthSize.width, CV_32F);
    m_depthMapBuffer = cv::Mat(m_depthSize.height, m_depthSize.width, CV_32F);
    m_depthMapReady = false;

    m_pointCloud = cv::Mat(m_depthSize.height, m_depthSize.width, CV_32FC3);
    m_pointCloudBuffer = cv::Mat(m_depthSize.height, m_depthSize.width, CV_32FC3);
    m_points = new CameraSpacePoint[m_depthSize.width * m_depthSize.height];

    m_depthFrameReader->SubscribeFrameArrived((WAITABLE_HANDLE*)&m_depthEventHandle);

    m_sensor->get_CoordinateMapper(&m_mapper);

    m_terminate = false;
    m_processThread = new std::thread(std::bind(&DepthCameraKinectSDK2::processDepth, this));

    return true;
}

void DepthCameraKinectSDK2::close()
{
    if (m_processThread) {
        m_terminate = true;
        m_processThread->join();
        delete m_processThread;
    }

    m_sensor->Close();
    m_sensor->Release();
    delete[] m_points;
}

void DepthCameraKinectSDK2::processDepth()
{
    while (!m_terminate) {
        if (WaitForSingleObject(m_depthEventHandle, INFINITE) == WAIT_OBJECT_0) {
            IDepthFrameArrivedEventArgs* eventArgs;
            if (SUCCEEDED(m_depthFrameReader->GetFrameArrivedEventData((WAITABLE_HANDLE)m_depthEventHandle, &eventArgs) && eventArgs)) {
                IDepthFrameReference* ref;
                if (SUCCEEDED(eventArgs->get_FrameReference(&ref)) && ref) {
                    IDepthFrame* frame;
                    if (SUCCEEDED(ref->AcquireFrame(&frame)) && frame) {
                        UINT capacity;
                        UINT16* buffer;
                        frame->AccessUnderlyingBuffer(&capacity, &buffer);

                        m_mapper->MapDepthFrameToCameraSpace(capacity, buffer, capacity, m_points);

                        CameraSpacePoint* pointsBufferRun = m_points;
                        UINT16* bufferRun = buffer;

                        std::unique_lock<std::mutex> lock(m_depthMapMutex);

                        m_depthMapBuffer.setTo(0);
                        m_pointCloudBuffer.setTo(0);

                        for (int i = 0; i < m_depthMapBuffer.rows; i++) {
                            float* depthRow = m_depthMapBuffer.ptr<float>(i);
                            cv::Vec3f* pointCloudRow = m_pointCloudBuffer.ptr<cv::Vec3f>(i);
                            for (int j = 0; j < m_depthMapBuffer.cols; j++) {
                                // set depth value
                                float depth = *bufferRun / 1000.0f;
                                depthRow[j] = depth;

                                // set point in point cloud
                                pointCloudRow[j] = cv::Vec3f((*pointsBufferRun).X, (*pointsBufferRun).Y, (*pointsBufferRun).Z);

                                bufferRun++;
                                pointsBufferRun++;
                            }
                        }

                        m_depthMapReady = true;
                        m_depthMapReadyCond.notify_all();
                        lock.unlock();

                        frame->Release();
                    }
                }
            }
        }
    }
}

float DepthCameraKinectSDK2::getMinReliableDistance() const
{
    return m_minReliableDistance;
}

float DepthCameraKinectSDK2::getMaxReliableDistance() const
{
    return m_maxReliableDistance;
}

float DepthCameraKinectSDK2::getFOVHorizontal() const
{
    return m_fovHorizontal;
}

float DepthCameraKinectSDK2::getFOVVertical() const
{
    return m_fovVertical;
}

float DepthCameraKinectSDK2::getFOVDiagonal() const
{
    return m_fovDiagonal;
}

cv::Size DepthCameraKinectSDK2::getDepthSize() const
{
    return m_depthSize;
}

void DepthCameraKinectSDK2::iWaitForData()
{
    std::unique_lock<std::mutex> lock(m_depthMapMutex);
    while (!m_depthMapReady)
        m_depthMapReadyCond.wait(lock);

    m_depthMapBuffer.copyTo(m_depthMap);
    m_pointCloudBuffer.copyTo(m_pointCloud);

    m_depthMapReady = false;
}
}
