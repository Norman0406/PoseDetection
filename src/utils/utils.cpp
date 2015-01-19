#include "utils.h"
#include "boundingbox3d.h"
#include <iostream>

namespace pose
{
void Utils::getColoredLabelMap(const cv::Mat& labelMap, cv::Mat& coloredLabelMap)
{
    if (coloredLabelMap.empty() || coloredLabelMap.rows != labelMap.rows ||
            coloredLabelMap.cols != labelMap.cols || coloredLabelMap.type() != CV_8UC3) {
        coloredLabelMap = cv::Mat(labelMap.rows, labelMap.cols, CV_8UC3);
    }
    coloredLabelMap.setTo(0);

    for (int i = 0; i < labelMap.cols; i++) {
        for (int j = 0; j < labelMap.rows; j++) {
            const unsigned int& label = labelMap.at<unsigned int>(cv::Point(i, j));

            if (label > 0) {
                cv::Vec3b color = getLabelColor(label);
                coloredLabelMap.at<cv::Vec3b>(cv::Point(i, j))[0] = (uchar)color[0];
                coloredLabelMap.at<cv::Vec3b>(cv::Point(i, j))[1] = (uchar)color[1];
                coloredLabelMap.at<cv::Vec3b>(cv::Point(i, j))[2] = (uchar)color[2];
            }
        }
    }
}

cv::Mat Utils::getColoredLabelMap(const cv::Mat& labelMap)
{
    cv::Mat coloredLabelMap(labelMap.rows, labelMap.cols, CV_8UC3);
    coloredLabelMap.setTo(0);

    getColoredLabelMap(labelMap, coloredLabelMap);

    return coloredLabelMap;
}

cv::Vec3b Utils::getLabelColor(unsigned int label)
{
    srand(label);
    return cv::Vec3b((rand() / (float)RAND_MAX) * 255,
                     (rand() / (float)RAND_MAX) * 255,
                     (rand() / (float)RAND_MAX) * 255);
}

bool Utils::loadCvMat(const char* filename, cv::Mat& image)
{
    if (!filename || strlen(filename) == 0)
        return false;

    FILE* file = NULL;

#ifdef _WIN32
    errno_t err = fopen_s(&file, filename, "rb");
    if(!file || err) {
        std::cerr << "could not open file: " << filename << std::endl;
        return false;
    }
#elif __APPLE__ & __MACH__
    file = fopen(filename, "rb");
    if(!file || ferror(file)) {
        cerr << "could not open file: " << filename << endl;
        return false;
    }
#endif

    // read header
    int flags = 0;
    int headerSize = 0;
    cv::Size imgSize(0, 0);
    int dims = 0;
    cv::Size origSize(0, 0);
    cv::Point startPoint(0, 0);
    int size = 0;

    char buffer[4];
    fread(&buffer, sizeof(char), 3, file);
    buffer[3] = '\0';

    if (strcmp(buffer, "CVM") != 0) {
        std::cerr << "file does not have cvm format: " << filename << std::endl;
        fclose(file);
        return 0;
    }

    // read header
    fread(&headerSize, sizeof(int), 1, file);
    fread(&flags, sizeof(int), 1, file);
    fread(&imgSize.width, sizeof(int), 1, file);
    fread(&imgSize.height, sizeof(int), 1, file);
    fread(&dims, sizeof(int), 1, file);
    fread(&origSize.width, sizeof(int), 1, file);
    fread(&origSize.height, sizeof(int), 1, file);
    fread(&startPoint.x, sizeof(int), 1, file);
    fread(&startPoint.y, sizeof(int), 1, file);
    fread(&size, sizeof(int), 1, file);

    image = cv::Mat(origSize.height, origSize.width, CV_MAT_TYPE(flags));

    // set file pointer
    fseek(file, headerSize, SEEK_SET);

    // read actual data
    fread(image.data, sizeof(unsigned char), size, file);

    // NOTE: Can be speeded up: don't create new image with the specified roi
    // like here, but only set image.dataStart and image.dataEnd accordingly
    // to point to the image region specified by the roi. [9/15/2011 Norman]
    image = image(cv::Rect(startPoint.x, startPoint.y, imgSize.width, imgSize.height));

    fclose(file);

    return true;
}

bool Utils::saveCvMat(const char* filename, const cv::Mat& image)
{
    if (!filename || strlen(filename) == 0)
        return false;

    FILE* file = NULL;

#ifdef _WIN32
    errno_t err = fopen_s(&file, filename, "wb");
    if(!file || err) {
        std::cerr << "could not create file: " << filename << std::endl;
        return false;
    }
#elif __APPLE__ & __MACH__
    file = fopen(filename, "wb");
    if(!file || ferror(file)) {
        cerr << "could not create file: " << filename << endl;
        return false;
    }
#endif

    // process roi information if available
    cv::Size origSize = cv::Size(0, 0);
    cv::Point startPoint = cv::Point(0, 0);

    image.locateROI(origSize, startPoint);
    int size = image.elemSize() * origSize.width * origSize.height;

    // header size
    const int headerSize = 3 * sizeof(char) +
        10 * sizeof(int);

    // write identification string
    fwrite("CVM", sizeof(char), 3, file);

    // write header
    fwrite(&headerSize, sizeof(int), 1, file);
    fwrite(&image.flags, sizeof(int), 1, file);
    fwrite(&image.cols, sizeof(int), 1, file);
    fwrite(&image.rows, sizeof(int), 1, file);
    fwrite(&image.dims, sizeof(int), 1, file);
    fwrite(&origSize.width, sizeof(int), 1, file);
    fwrite(&origSize.height, sizeof(int), 1, file);
    fwrite(&startPoint.x, sizeof(int), 1, file);
    fwrite(&startPoint.y, sizeof(int), 1, file);
    fwrite(&size, sizeof(int), 1, file);

    // write data
    fwrite(image.data, sizeof(uchar), size, file);

    fclose(file);

    return true;
}

float distancePrio1(const BoundingBox3D& box1, const BoundingBox3D& box2)
{
    float distLeft = box1.getAnchorDistance(BoundingBox3D::AT_LEFT, box2);
    float distTop = box1.getAnchorDistance(BoundingBox3D::AT_TOP, box2);
    float distRight = box1.getAnchorDistance(BoundingBox3D::AT_RIGHT, box2);
    float distBottom = box1.getAnchorDistance(BoundingBox3D::AT_BOTTOM, box2);

    return sqrtf(distLeft * distLeft + distTop * distTop + distRight * distRight + distBottom * distBottom);
    //return (distLeft + distTop + distRight + distBottom) / 4.0f;
    //return distLeft + distTop + distRight + distBottom;
}

float distancePrio2(const BoundingBox3D& box1, const BoundingBox3D& box2)
{
    float distLeft = box1.getAnchorDistance(BoundingBox3D::AT_LEFT, box2);
    float distTop = box1.getAnchorDistance(BoundingBox3D::AT_TOP, box2);
    float distRight = box1.getAnchorDistance(BoundingBox3D::AT_RIGHT, box2);
    float distBottom = box1.getAnchorDistance(BoundingBox3D::AT_BOTTOM, box2);

    float d1 = sqrtf(distLeft * distLeft + distTop * distTop + distRight * distRight);
    float d2 = sqrtf(distLeft * distLeft + distRight * distRight + distBottom * distBottom);
    float d3 = sqrtf(distTop * distTop + distRight * distRight + distBottom * distBottom);

    return std::min(d1, std::min(d2, d3));
}

float distancePrio3(const BoundingBox3D& box1, const BoundingBox3D& box2)
{
    float distLeft = box1.getAnchorDistance(BoundingBox3D::AT_LEFT, box2);
    float distTop = box1.getAnchorDistance(BoundingBox3D::AT_TOP, box2);
    float distRight = box1.getAnchorDistance(BoundingBox3D::AT_RIGHT, box2);
    float distBottom = box1.getAnchorDistance(BoundingBox3D::AT_BOTTOM, box2);

    float d1 = sqrtf(distLeft * distLeft + distTop * distTop);
    float d2 = sqrtf(distLeft * distLeft + distRight * distRight);
    float d3 = sqrtf(distLeft * distLeft + distBottom * distBottom);
    float d4 = sqrtf(distTop * distTop + distRight * distRight);
    //float d5 = sqrtf(distTop * distTop + distBottom * distBottom);
    float d6 = sqrtf(distRight * distRight + distBottom * distBottom);

    //return std::min(d1, std::min(d2, std::min(d3, std::min(d4, std::min(d5, d6)))));
    return std::min(d1, std::min(d2, std::min(d3, std::min(d4, d6))));
}

float distancePrio4(const BoundingBox3D& box1, const BoundingBox3D& box2)
{
    float distLeft = box1.getAnchorDistance(BoundingBox3D::AT_LEFT, box2);
    float distTop = box1.getAnchorDistance(BoundingBox3D::AT_TOP, box2);
    float distRight = box1.getAnchorDistance(BoundingBox3D::AT_RIGHT, box2);
    float distBottom = box1.getAnchorDistance(BoundingBox3D::AT_BOTTOM, box2);

    return std::min(distLeft, std::min(distTop, std::min(distRight, distBottom)));
}

float Utils::distance(const BoundingBox3D& box1, const BoundingBox3D& box2, float searchRadius)
{
    /* priorisierung:
     * 1.)  AT_LEFT + AT_TOP + AT_RIGHT + AT_BOTTOM
     *
     * 2.)  AT_LEFT + AT_TOP + AT_RIGHT
     *      AT_LEFT + AT_RIGHT + AT_BOTTOM
     *      AT_TOP + AT_RIGHT + AT_BOTTOM
     *
     * 3.)  AT_LEFT + AT_TOP
     *      AT_LEFT + AT_RIGHT
     *      AT_LEFT + AT_BOTTOM
     *      AT_TOP + AT_RIGHT
     *      AT_TOP + AT_BOTTOM
     *      AT_RIGHT + AT_BOTTOM
     *
     * 4.)  AT_LEFT
     *      AT_TOP
     *      AT_RIGHT
     *      AT_BOTTOM
     */

    float dist1 = distancePrio1(box1, box2);
    /*if (dist1 < searchRadius)
        return dist1;*/

    float dist2 = distancePrio2(box1, box2);
    /*if (dist2 < searchRadius)
        return dist2;*/

    float dist3 = distancePrio3(box1, box2);
    /*if (dist3 < searchRadius)
        return dist3;*/

    //float dist4 = distancePrio4(box1, box2);
    return std::min(dist1, std::min(dist2, dist3));
}

cv::Point2f Utils::projectPoint(const cv::Point3f& point, const cv::Mat& projectionMatrix)
{
    cv::Mat pointMat(4, 1, CV_32F);
    pointMat.ptr<float>(0)[0] = point.x;
    pointMat.ptr<float>(0)[1] = point.y;
    pointMat.ptr<float>(0)[2] = point.z;
    pointMat.ptr<float>(0)[3] = 1;

    // compute image position
    cv::Mat pointImgMat = projectionMatrix * pointMat;
    return cv::Point2f(pointImgMat.ptr<float>(0)[0] / pointImgMat.ptr<float>(0)[2],
            pointImgMat.ptr<float>(0)[1] / pointImgMat.ptr<float>(0)[2]);
}

/*void Utils::matrix2Quat(const double* rot, double* quat)
{
    // convert rotation matrix to quaternion (adapted from OgreQuaternion.cpp)

    double matrix[3][3] = {
        {rot[0], rot[1], rot[2]},
        {rot[3], rot[4], rot[5]},
        {rot[6], rot[7], rot[8]}
    };

    double trace = matrix[0][0] + matrix[1][1] + matrix[2][2];
    double root;

    if (trace > 0.0) {
        root = sqrt(trace + 1.0);
        quat[3] = 0.5 * root;
        root = 0.5 / root;

        quat[0] = (matrix[2][1] - matrix[1][2]) * root;
        quat[1] = (matrix[0][2] - matrix[2][0]) * root;
        quat[2] = (matrix[1][0] - matrix[0][1]) * root;
    }
    else {
        static size_t next[3] = {1, 2, 0};
        size_t i = 0;
        if (matrix[1][1] > matrix[0][0])
            i = 1;
        if (matrix[2][2] > matrix[i][i])
            i = 2;
        size_t j = next[i];
        size_t k = next[j];

        root = sqrt(matrix[i][i] - matrix[j][j] - matrix[k][k] + 1.0);
        double* apkQuat[3] = {&quat[0], &quat[1], &quat[2]};
        *apkQuat[i] = 0.5 * root;
        root = 0.5 / root;
        quat[3] = (matrix[k][j] - matrix[j][k]) * root;
        *apkQuat[j] = (matrix[j][i] + matrix[i][j]) * root;
        *apkQuat[k] = (matrix[k][i] + matrix[i][k]) * root;
    }
}

void Utils::matrix2Quat(const cv::Mat& rot, quaternion<double>& quat)
{
    if (rot.type() != CV_64F)
        throw Exception("invalid type");

    double q[4] = { quat.R_component_2(),
        quat.R_component_3(),
        quat.R_component_4(),
        quat.R_component_1() };

    matrix2Quat((double*)&rot.data[0], q);

    quat = boost::math::quaternion<double>(q[3], q[0], q[1], q[2]);
}

void Utils::quat2Matrix(const double* quat, double* rot)
{
    // convert quaternion to rotation matrix (adapted from OgreQuaternion.cpp)

    double tx = quat[0] + quat[0];
    double ty = quat[1] + quat[1];
    double tz = quat[2] + quat[2];
    double twx = tx * quat[3];
    double twy = ty * quat[3];
    double twz = tz * quat[3];
    double txx = tx * quat[0];
    double txy = ty * quat[0];
    double txz = tz * quat[0];
    double tyy = ty * quat[1];
    double tyz = tz * quat[1];
    double tzz = tz * quat[2];

    double matrix[3][3];

    matrix[0][0] = 1.0 - (tyy + tzz);
    matrix[0][1] = txy - twz;
    matrix[0][2] = txz + twy;
    matrix[1][0] = txy + twz;
    matrix[1][1] = 1.0 - (txx + tzz);
    matrix[1][2] = tyz - twx;
    matrix[2][0] = txz - twy;
    matrix[2][1] = tyz + twx;
    matrix[2][2] = 1.0 - (txx + tyy);

    rot[0] = matrix[0][0];
    rot[1] = matrix[0][1];
    rot[2] = matrix[0][2];
    rot[3] = matrix[1][0];
    rot[4] = matrix[1][1];
    rot[5] = matrix[1][2];
    rot[6] = matrix[2][0];
    rot[7] = matrix[2][1];
    rot[8] = matrix[2][2];
}

void Utils::quat2Matrix(const quaternion<double>& quat, cv::Mat& rot)
{
    if (!rot.empty() && rot.type() != CV_64F)
        throw Exception("invalid type");
    else
        rot = cv::Mat(3, 3, CV_64FC1);

    double q[4] = { quat.R_component_2(),
        quat.R_component_3(),
        quat.R_component_4(),
        quat.R_component_1() };

    quat2Matrix(q, (double*)&rot.data[0]);
}

void Utils::euler2Quat(quaternion<double>& quat, double angleX, double angleY, double angleZ)
{
    double r = DEG2RAD(angleX / 2.0);
    double p = DEG2RAD(angleY / 2.0);
    double y = DEG2RAD(angleZ / 2.0);

    double sinp = sin(p);
    double siny = sin(y);
    double sinr = sin(r);
    double cosp = cos(p);
    double cosy = cos(y);
    double cosr = cos(r);

    quat = quaternion<double>(
        cosr * cosp * cosy + sinr * sinp * siny,
        sinr * cosp * cosy - cosr * sinp * siny,
        cosr * sinp * cosy + sinr * cosp * siny,
        cosr * cosp * siny - sinr * sinp * cosy);

    // normalize
    quat /= norm(quat);
}

void Utils::quat2Euler(const quaternion<double>& quat, double& angleX, double& angleY, double& angleZ)
{
    quaternion<double> myQuat = quat;
    myQuat /= norm(myQuat);

    cv::Point3d euler(0, 0, 0);

    double qW = myQuat.R_component_1();
    double qX = myQuat.R_component_2();
    double qY = myQuat.R_component_3();
    double qZ = myQuat.R_component_4();

    double test = (qW * qY - qZ * qX);
    double unit = qX * qX + qY * qY + qZ * qZ + qW * qW;

    // handle singularities
    if (test > 0.4999999 * unit) {
        euler.x = 2 * atan2(qX, qW);
        euler.y = M_PI / 2.0;
        euler.z = 0;
    }
    else if (test < -0.4999999 * unit) {
        euler.x = 2 * atan2(qX, qW);
        euler.y = -M_PI / 2.0;
        euler.z = 0;
    }
    else {
        euler.x = atan2(2 * (qW * qX + qY * qZ), 1 - 2 * (qX * qX + qY * qY));
        euler.y = asin(2 * test);
        euler.z = atan2(2 * (qW * qZ + qX * qY), 1 - 2 * (qY * qY + qZ * qZ));
    }

    angleX = RAD2DEG(euler.x);
    angleY = RAD2DEG(euler.y);
    angleZ = RAD2DEG(euler.z);
}

void Utils::axis2Quat(quaternion<double>& quat, const cv::Point3d& axis, double angle)
{
    cv::Point3d myAxis = axis;
    myAxis *= (1.0 / cv::norm(myAxis));

    double halfAngle = DEG2RAD(angle / 2.0);
    double sinAngle = sin(halfAngle);

    quat = quaternion<double>(cos(halfAngle),
        myAxis.x * sinAngle, myAxis.y * sinAngle, myAxis.z * sinAngle);
}

void Utils::quat2Axis(const quaternion<double>& quat, cv::Point3d& axis, double& angle)
{
    quaternion<double> myQuat = quat;
    double qw = myQuat.R_component_1();

    // normalize
    if (qw > 1.0)
        myQuat /= norm(myQuat);

    angle = RAD2DEG(2.0 * acos(qw));
    double s = sqrt(1.0 - qw * qw);

    if (s < 0.0001) {
        // avoid divbyzero, any arbitrary axis is valid
        axis.x = 0;
        axis.y = 1;
        axis.z = 0;
    }
    else {
        axis.x = myQuat.R_component_2() / s;
        axis.y = myQuat.R_component_3() / s;
        axis.z = myQuat.R_component_4() / s;
    }
}

void Utils::quatRotate(const quaternion<double>& quat, cv::Point3d& point)
{
    quaternion<double> pointTemp(0, point.x, point.y, point.z);
    pointTemp = conj(quat) * pointTemp * quat;
    point = cv::Point3d(pointTemp.R_component_2(), pointTemp.R_component_3(),
        pointTemp.R_component_4());
}

void Utils::quatRotateInv(const Eigen::Quaterniond& quat, cv::Point3d& point)
{
    quaternion<double> pointTemp(0, point.x, point.y, point.z);
    pointTemp = quat * pointTemp * conj(quat);
    point = cv::Point3d(pointTemp.R_component_2(), pointTemp.R_component_3(),
        pointTemp.R_component_4());
}*/
}
