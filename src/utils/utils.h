#ifndef UTILS_H
#define UTILS_H

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace pose
{
class BoundingBox3D;

#define UNUSED(x) (void)(x)
#define DEG2RAD(a) (a * (M_PI / 180.0))
#define RAD2DEG(a) ((a * 180.0) / M_PI)

class Utils
{
public:
    static void getColoredLabelMap(const cv::Mat&, cv::Mat&);
    static cv::Mat getColoredLabelMap(const cv::Mat&);
    static cv::Scalar getLabelColor(unsigned int label);
    static bool loadCvMat(const char* filename, cv::Mat& image);
    static bool saveCvMat(const char* filename, const cv::Mat& image);
    static float distance(const BoundingBox3D& box1, const BoundingBox3D& box2, float searchRadius);
    static cv::Point2f projectPoint(const cv::Point3f& point, const cv::Mat& projectionMatrix);

    /*static void matrix2Quat(const double* rot, double* quat);
    static void matrix2Quat(const Eigen::Matrix& rot, Eigen::Quaterniond& quat);
    static void quat2Matrix(const double* quat, double* rot);
    static void quat2Matrix(const Eigen::Quaterniond& quat, Eigen::Matrix& rot);
    static void euler2Quat(Eigen::Quaterniond& quat, double angleX, double angleY, double angleZ);
    static void quat2Euler(const Eigen::Quaterniond& quat, double& angleX, double& angleY, double& angleZ);
    static void axis2Quat(Eigen::Quaterniond& quat, const cv::Point3d& axis, double angle);
    static void quat2Axis(const Eigen::Quaterniond& quat, cv::Point3d& axis, double& angle);
    static void quatRotate(const Eigen::Quaterniond& quat, cv::Point3d& point);
    static void quatRotateInv(const Eigen::Quaterniond& quat, cv::Point3d& point);*/

private:
    Utils();
    ~Utils();
};
}

#endif // LABELING_H
