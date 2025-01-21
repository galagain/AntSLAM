#include "MotionEstimator.h"

MotionEstimator::MotionEstimator(double focal, const cv::Point2d &pp)
    : focal(focal), pp(pp) {}

void MotionEstimator::estimateMotion(const std::vector<cv::Point2f> &points1,
                                     const std::vector<cv::Point2f> &points2,
                                     cv::Mat &R, cv::Mat &t) const
{
    cv::Mat E, mask;

    // Compute the essential matrix using RANSAC
    E = cv::findEssentialMat(points2, points1, focal, pp, cv::RANSAC, 0.999, 1.0, mask);

    // Recover the rotation and translation matrices from the essential matrix
    cv::recoverPose(E, points2, points1, R, t, focal, pp, mask);
}
