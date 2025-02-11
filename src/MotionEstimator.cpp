/// @file MotionEstimator.cpp
/// @brief Implements the MotionEstimator class for estimating camera motion.

#include "MotionEstimator.h"

/**
 * @brief Constructor initializing camera intrinsic parameters.
 * @param focal Camera focal length.
 * @param pp Principal point of the camera.
 */
MotionEstimator::MotionEstimator(double focal, const cv::Point2d &pp)
    : focal(focal), pp(pp) {}

/**
 * @brief Estimates motion (rotation & translation) from corresponding feature points.
 * @param points1 Feature points in the first image.
 * @param points2 Corresponding feature points in the second image.
 * @param R Output rotation matrix.
 * @param t Output translation vector.
 */
void MotionEstimator::estimateMotion(const std::vector<cv::Point2f> &points1,
                                     const std::vector<cv::Point2f> &points2,
                                     cv::Mat &R, cv::Mat &t) const
{
    cv::Mat E, mask;
    E = cv::findEssentialMat(points2, points1, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
    cv::recoverPose(E, points2, points1, R, t, focal, pp, mask);
}
