#include "input.h"
#include <utils/exception.h>

namespace pose
{
Input::Input(int width, int height)
    : m_width(width),
      m_height(height)
{
    m_depthMap = cv::Mat(m_height, m_width, CV_32F);
    m_pointCloud = cv::Mat(m_height, m_width, CV_32FC3);
}

Input::~Input()
{
    m_depthMap.release();
    m_pointCloud.release();
}

void Input::process(const float* depthData, int depthDataSize, const float* pointsData, int pointsDataSize)
{
    // check data sizes
    if (depthDataSize * sizeof(float) != m_depthMap.cols * m_depthMap.rows * m_depthMap.elemSize() ||
        pointsDataSize * sizeof(float) != m_pointCloud.cols * m_pointCloud.rows * m_pointCloud.elemSize())
        throw Exception("invalid input data size(s)");

    // copy data
    memcpy(m_depthMap.data, depthData, depthDataSize * sizeof(float));
    memcpy(m_pointCloud.data, pointsData, pointsDataSize * sizeof(float));

    // compute projection matrix
    if (!m_pointCloud.empty() && m_projectionMatrix.empty())
        m_projectionMatrix = computeProjectionMatrix(m_pointCloud);
}

const bool Input::ready() const
{
    return !m_depthMap.empty() && !m_pointCloud.empty() && !m_projectionMatrix.empty();
}

const cv::Mat& Input::getDepthMap() const
{
    return m_depthMap;
}

const cv::Mat& Input::getPointCloud() const
{
    return m_pointCloud;
}

const cv::Mat& Input::getProjectionMatrix() const
{
    return m_projectionMatrix;
}

cv::Mat Input::computeProjectionMatrix(const cv::Mat& pointCloud) const
{
    if (pointCloud.empty() || pointCloud.type() != CV_32FC3)
        return cv::Mat();

    const int numCorrespondences = 100;
    std::vector<std::vector<float> > objPoints;
    std::vector<std::vector<float> > imgPoints;
    objPoints.reserve(numCorrespondences);
    imgPoints.reserve(numCorrespondences);

    const int maxNumIter = numCorrespondences * 2;

    int numIter = 0;

    // create point correspondences with random points and make sure there are
    // no duplicate points
    do {
        float val1 = rand() / (float)RAND_MAX;
        float val2 = rand() / (float)RAND_MAX;
        int x = (int)(val1 * pointCloud.cols);
        int y = (int)(val2 * pointCloud.rows);

        if (x < 0 || x >= pointCloud.cols ||
            y < 0 || y >= pointCloud.rows)
            continue;

        numIter++;

        const float* row = pointCloud.ptr<float>(y);

        // point correspondence: 3D world point
        std::vector<float> objPoint(4);
        objPoint[0] = row[x * 3 + 0];
        objPoint[1] = row[x * 3 + 1];
        objPoint[2] = row[x * 3 + 2];
        objPoint[3] = 1.0f;

        // the point is invalid (z value is 0), so don't consider it
        if (objPoint[2] <= 0)
            continue;

        // point correspondence: 2D image point
        std::vector<float> imgPoint(3);
        imgPoint[0] = (float)x;
        imgPoint[1] = (float)y;
        imgPoint[2] = 1.0f;

        // if this point is already in the list, go on to the next iteration
        if (std::find(imgPoints.begin(), imgPoints.end(), imgPoint) != imgPoints.end())
            continue;

        // add the point correspondence
        objPoints.push_back(objPoint);
        imgPoints.push_back(imgPoint);
    } while (imgPoints.size() < numCorrespondences && numIter <= maxNumIter);

    // there were not enough points, try again on the next image
    if (numIter > maxNumIter)
        return cv::Mat();

    // reconstruction from given point correspondences
    // (see http://de.wikipedia.org/wiki/Projektionsmatrix#Berechnung_der_Projektionsmatrix_aus_Punktkorrespondenzen)

    // create Matrix A
    cv::Mat matA(2 * numCorrespondences, 12, CV_32FC1);
    for (int i = 0, j = 0; i < 2 * numCorrespondences; i += 2, j++) {	// cols
        // first row
        matA.ptr<float>(i)[0] = objPoints[j][0];
        matA.ptr<float>(i)[1] = objPoints[j][1];
        matA.ptr<float>(i)[2] = objPoints[j][2];
        matA.ptr<float>(i)[3] = objPoints[j][3];

        matA.ptr<float>(i)[4] = matA.ptr<float>(i)[5] =
            matA.ptr<float>(i)[6] = matA.ptr<float>(i)[7] = 0;

        matA.ptr<float>(i)[8] = - imgPoints[j][0] * objPoints[j][0];
        matA.ptr<float>(i)[9] = - imgPoints[j][0] * objPoints[j][1];
        matA.ptr<float>(i)[10] = - imgPoints[j][0] * objPoints[j][2];
        matA.ptr<float>(i)[11] = - imgPoints[j][0] * objPoints[j][3];

        // second row
        matA.ptr<float>(i+1)[0] = matA.ptr<float>(i+1)[1] =
            matA.ptr<float>(i+1)[2] = matA.ptr<float>(i+1)[3] = 0;

        matA.ptr<float>(i+1)[4] = objPoints[j][0];
        matA.ptr<float>(i+1)[5] = objPoints[j][1];
        matA.ptr<float>(i+1)[6] = objPoints[j][2];
        matA.ptr<float>(i+1)[7] = objPoints[j][3];

        matA.ptr<float>(i+1)[8] = - imgPoints[j][1] * objPoints[j][0];
        matA.ptr<float>(i+1)[9] = - imgPoints[j][1] * objPoints[j][1];
        matA.ptr<float>(i+1)[10] = - imgPoints[j][1] * objPoints[j][2];
        matA.ptr<float>(i+1)[11] = - imgPoints[j][1] * objPoints[j][3];
    }

    // solve using SVD
    cv::Mat w;
    cv::Mat u;
    cv::Mat vt;

    cv::SVD mySVD;
    mySVD.compute(matA, w, u, vt);

    // create matrix
    cv::Mat projectionMatrix(3, 4, CV_32FC1);
    for (int i = 0; i < vt.rows; i++) {
        projectionMatrix.ptr<float>(0)[i] = vt.ptr<float>(vt.cols - 1)[i] /
            vt.ptr<float>(vt.cols - 1)[vt.rows - 1];
    }

    // normalize projection matrix
    float sum = projectionMatrix.ptr<float>(2)[0] +
        projectionMatrix.ptr<float>(2)[1] +
        projectionMatrix.ptr<float>(2)[2];
    projectionMatrix = projectionMatrix * (1.0f / sum);

    return projectionMatrix;
}
}
