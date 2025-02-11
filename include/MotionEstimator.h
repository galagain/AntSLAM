/// @file MotionEstimator.h
/// @brief Defines the MotionEstimator class for estimating camera motion.

#ifndef MOTIONESTIMATOR_H
#define MOTIONESTIMATOR_H

#include <opencv2/opencv.hpp>
#include <vector>

/**
 * @class MotionEstimator
 * @brief Estimates camera motion using feature correspondences.
 */
class MotionEstimator
{
public:
    /**
     * @brief Constructor initializing camera parameters.
     * @param focal Camera focal length.
     * @param pp Principal point of the camera.
     */
    MotionEstimator(double focal, const cv::Point2d &pp);

    /**
     * @brief Estimates motion from feature correspondences.
     * @param points1 Feature points in the first image.
     * @param points2 Corresponding feature points in the second image.
     * @param R Output rotation matrix.
     * @param t Output translation matrix.
     */
    void estimateMotion(const std::vector<cv::Point2f> &points1,
                        const std::vector<cv::Point2f> &points2,
                        cv::Mat &R, cv::Mat &t) const;

private:
    double focal;   ///< Camera focal length.
    cv::Point2d pp; ///< Principal point.
};

#endif // MOTIONESTIMATOR_H
