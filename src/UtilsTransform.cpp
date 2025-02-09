#include "UtilsTransform.h"

cv::Mat UtilsTransform::xyzToTransformMatrix(const cv::Point3f &xyz)
{
    cv::Mat transform = cv::Mat::eye(4, 4, CV_64F);
    transform.at<double>(0, 3) = xyz.x;
    transform.at<double>(1, 3) = xyz.y;
    transform.at<double>(2, 3) = xyz.z;
    return transform;
}

cv::Point3f UtilsTransform::transformMatrixToXYZ(const cv::Mat &transform)
{
    return cv::Point3f(
        transform.at<double>(0, 3),
        transform.at<double>(1, 3),
        transform.at<double>(2, 3));
}

Eigen::Quaterniond UtilsTransform::rotationMatrixToQuaternion(const cv::Mat &rotation)
{
    Eigen::Matrix3d rot;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            rot(i, j) = rotation.at<double>(i, j);
        }
    }
    return Eigen::Quaterniond(rot);
}

cv::Mat UtilsTransform::quaternionToRotationMatrix(const Eigen::Quaterniond &quat)
{
    Eigen::Matrix3d rot = quat.toRotationMatrix();
    cv::Mat rotation(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            rotation.at<double>(i, j) = rot(i, j);
        }
    }
    return rotation;
}
