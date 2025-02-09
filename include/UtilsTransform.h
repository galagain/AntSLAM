#ifndef UTILS_TRANSFORM_H
#define UTILS_TRANSFORM_H

#include <opencv2/core.hpp>
#include <vector>
#include <Eigen/Dense>

class UtilsTransform
{
public:
    // Convert XYZ translation to a 4x4 transformation matrix
    static cv::Mat xyzToTransformMatrix(const cv::Point3f &xyz);

    // Convert a 4x4 transformation matrix to XYZ translation
    static cv::Point3f transformMatrixToXYZ(const cv::Mat &transform);

    // Convert a 3x3 rotation matrix to quaternion
    static Eigen::Quaterniond rotationMatrixToQuaternion(const cv::Mat &rotation);

    // Convert a quaternion to a 3x3 rotation matrix
    static cv::Mat quaternionToRotationMatrix(const Eigen::Quaterniond &quat);
};

#endif // UTILS_TRANSFORM_H
